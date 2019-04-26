#include "Ledger.hpp"
#include "ImpliedTransaction.hpp"
#include "Utils.hpp"
#include <map>
#include <fstream>

using namespace std;
using namespace lot49;

//
// Simplified ledger that spends existing outputs to create new outputs in the UTXO set.
// An existing output can be spent if the new output is signed by the public key
//   of the input owner(s).
//

// declare static instance  
Ledger Ledger::sInstance;

void Ledger::Issue(const bls::PublicKey& inOwner, const uint16_t inAmount, const uint16_t inCount)
{
    // create some outputs for this owner
    for (int i = 0; i < inCount; i++) {

        // create a new issued output
        ImpliedTransaction issued_output = ImpliedTransaction::Issue(inOwner, inAmount);

        mUTXOs[issued_output.GetHash()] = issued_output;
    }
}

const std::vector<ImpliedTransaction> Ledger::Sort(const std::vector<ImpliedTransaction>& inTransactions)
{
    std::vector<ImpliedTransaction> sorted_transactions = inTransactions;

    // sort by transaction type
    std::sort(sorted_transactions.begin(), sorted_transactions.end(), [](ImpliedTransaction& a, ImpliedTransaction& b) {
        return a.GetType() < b.GetType(); });

    return sorted_transactions;
}

// find input transaction for
bool Ledger::FindInputTransaction(const ImpliedTransaction& inTransaction, const TransactionMap& inProposedUTXOs, ImpliedTransaction& outInput) 
{
    // find input for the transaction in the UTXO set
    TransactionMap::const_iterator input_iter;
    if (inTransaction.GetType() == eSetup) {
        input_iter = Find(inTransaction.GetSigner(), inTransaction.GetOutputAmount(0));
    }
    else {
        input_iter = Find(inTransaction.GetInputHash());
    }

    if (input_iter != mUTXOs.end()) {
        _log << "\tFOUND Input tx id in UTXO set: " << std::hex << input_iter->second.GetId() << endl;
    }
    else {
        // check utxo proposed in the current set of transacctions
        input_iter = inProposedUTXOs.find(inTransaction.GetInputHash());

        if (input_iter == inProposedUTXOs.end()) {
            // skip transactions with missing inputs
            _log << "\tInput tx id NOT FOUND: " << std::hex << inTransaction.GetInputId() << endl;
            return false;
        }
        _log << "\tFOUND Input tx id in proposed UTXO set: " << std::hex << input_iter->second.GetId() << endl;
    }

    // return input transaction consumed by inTransaction
    outInput = input_iter->second;

    return true;
}

// true if value from input earmarked for output is sufficient to fund output
bool Ledger::ValidTransfer(const ImpliedTransaction& inInput, const ImpliedTransaction& inOutput) const
{
    bool isValid = true;
    if (inInput.IsMultisig() && inOutput.IsMultisig()) {
        // multisig -> multsig
        isValid &= (inInput.GetOutputOwner(0) == inOutput.GetOutputOwner(0));
        isValid &= (inInput.GetOutputOwner(1) == inOutput.GetOutputOwner(1));
    }
    else if (!inOutput.IsMultisig()) {
        // multisig -> single owner
        if (inInput.GetOutputOwner(0) == inOutput.GetOutputOwner(0)) {
            isValid &= (inInput.GetOutputAmount(0) >= inOutput.GetOutputAmount());
        }
        else if (inInput.GetOutputOwner(1) == inOutput.GetOutputOwner(0)) {
            isValid &= (inInput.GetOutputAmount(1) >= inOutput.GetOutputAmount());
        }
        else {
            isValid = false;
        }
    }
    
    isValid &= (inInput.GetOutputAmount() >= inOutput.GetOutputAmount());
    return isValid;
}

bool Ledger::Add(const std::vector<ImpliedTransaction>& inTransactions)
{
    _log << "Ledger::Add Transactions:" << endl << endl;

    //_log << "UTXOs:" << endl;
    //_log << "Transaction Id\tType\tSigner PK\tOutput Amount 1\tOutput Amount 2" << endl;
    //for (auto& tx_iter : mUTXOs) {
    //    _log << std::hex << tx_iter.second.GetId() << "\t" << tx_iter.second.GetType() << "\t" << tx_iter.second.GetSigner() << "\t" << tx_iter.second.GetOutputAmount(0) << "\t" << tx_iter.second.GetOutputAmount(1) << endl;
    //}

    // sort transactions in order of dependence
    std::vector<ImpliedTransaction> sorted_transations = Sort(inTransactions);

    _log << "\tSorted Transactions:" << endl;
    _log << "\tTransaction Id\tType\tSigner PK\tOutput Amount 1\tOutput Amount 2" << endl;
    for (auto& tx_iter : sorted_transations) {
        _log << "\t" << std::hex << tx_iter.GetId() << "\t" << tx_iter.GetType() << "\t" << tx_iter.GetSigner() << "\t" << tx_iter.GetOutputAmount(0) << "\t" << tx_iter.GetOutputAmount(1) << endl;
    }

    // record which owners signed each new output transaction
    TransactionMap proposed_utxo;

    _log << "\tValid Transactions:" << endl;
    for (const auto& tx : sorted_transations) {
        _log << std::hex << tx.GetId() << "\t" << tx.GetType() << "\t" << tx.GetSigner() << "\t" << tx.GetOutputAmount(0) << "\t" << tx.GetOutputAmount(1) << endl;

        ImpliedTransaction input_tx;
        if (FindInputTransaction(tx, proposed_utxo, input_tx)) {

            // check which input owners signed the new output transaction to determine input amount
            if (!ValidTransfer(input_tx, tx)) {
                continue;
            }

            // if we found the input utxo for the new transaction, check if it has already been proposed as a new UTXO with a different signature
            auto output_iter = proposed_utxo.find(tx.GetHash());
            if (output_iter == proposed_utxo.end()) {
                // save newly proposed utxo
                proposed_utxo[tx.GetHash()] = tx;
            }
            else {
                // add second signer to proposed utxo that has already been signed by one output owner
                bool isValid = output_iter->second.AddSigner(tx.GetSigner());
                assert(isValid);
            }
        }
    }

    // add all proposed transactions (with valid inputs) to the utxo set
    int added = 0;
    int removed = 0;
    TransactionMap spent_txs;
    TransactionMap new_block;
    for (const auto& tx_iter : proposed_utxo) {
        
        // find UTXO input for the transaction
        const ImpliedTransaction tx = tx_iter.second;

        ImpliedTransaction input_tx;
        if (!FindInputTransaction(tx, proposed_utxo, input_tx)) {
            continue;
        }

        // must be signed by owner(s) of input tx
        if (input_tx.GetAggregateOutputOwner() != tx.GetSigner()) {
            continue;
        }

        // add used tx from the UTXO set to a list of txs to remove from the UTXO set
        if (mUTXOs.find(input_tx.GetHash()) != mUTXOs.end() && spent_txs.find(input_tx.GetHash()) == spent_txs.end()) {
            spent_txs[input_tx.GetHash()] = input_tx;
        }        

        // add valid utxo to new block
        new_block[tx.GetHash()] = tx;
    }

    // remove spent transactions from utxo set
    for (const auto& tx_iter : spent_txs) {
        TransactionMap::const_iterator find_iter = mUTXOs.find(tx_iter.first);
        assert(find_iter != mUTXOs.end());
        mUTXOs.erase(find_iter);
        removed++;
    }

    // add new transactions from block to utxo set
    for (const auto& tx_iter : new_block) {
        mUTXOs[tx_iter.first] = tx_iter.second;
        added++;
    }

    assert(added > 0);
    assert(removed > 0);
    return (added > 0);
}

bool Ledger::Unspent(const std::vector<uint8_t>& inTransactionHash) const
{
    Ledger::TransactionMap::const_iterator iter = Ledger::Find(inTransactionHash);
    return (iter != mUTXOs.end());
}

Ledger::TransactionMap::const_iterator Ledger::Find(const std::vector<uint8_t>& inTransactionHash) const
{
    // return true if transaction hash is in the UTXO set
    Ledger::TransactionMap::const_iterator iter = mUTXOs.find(inTransactionHash);
    return iter;
}

Ledger::TransactionMap::const_iterator Ledger::Find(const bls::PublicKey& inOwner, const uint16_t inAmount) const
{
    for (auto iter = mUTXOs.begin(); iter != mUTXOs.end(); iter++) {
        if (iter->second.GetAggregateOutputOwner() == inOwner) {
            uint16_t output_amount = iter->second.GetOutputAmount(0);
            if (iter->second.IsMultisig()) {
                output_amount += iter->second.GetOutputAmount(1);
            }
            if (output_amount >= inAmount) {
                return iter;
            }
        }
    }
    return mUTXOs.end();
}