#include "ImpliedTransaction.hpp"
#include <map>

#pragma once

//
// Simplified ledger that adds valid transactions to UTXO set; transactions are looked up based on public key
//   of the owner. A 2:2 multisig UTXO is looked up with an aggregate public key
//

namespace lot49
{

class Ledger {
    public:
    typedef std::map< std::vector<uint8_t>, ImpliedTransaction > TransactionMap;

    static Ledger sInstance;

    void Issue(const bls::PublicKey& inOwner, const uint16_t inAmount, const uint16_t inCount);
    bool Add(const std::vector<ImpliedTransaction>& inTransactions);
    bool Unspent(const std::vector<uint8_t>& inTransactionHash) const;

    private:

    static const std::vector<ImpliedTransaction> Sort(const std::vector<ImpliedTransaction>& inTransactions);

    // find input transaction for
    bool FindInputTransaction(const ImpliedTransaction& inTransaction, const TransactionMap& inProposedUTXOs, ImpliedTransaction& outInput);

    // true if value from input earmarked for output is sufficient to fund output
    bool ValidTransfer(const ImpliedTransaction& inInput, const ImpliedTransaction& inOutput) const;

    TransactionMap::const_iterator Find(const std::vector<uint8_t>& inTransactionHash) const;
    TransactionMap::const_iterator Find(const bls::PublicKey& inOwner, const uint16_t inAmount) const;

    // map from transaction hash of an unspent transaction output to the transaction itself
    TransactionMap mUTXOs;
};

}; // namespace lot49
