#include "bls.hpp"
#include "MeshNode.hpp"
#include "ImpliedTransaction.hpp"
#include <random>

static std::default_random_engine rng(std::random_device{}());
static std::uniform_int_distribution<uint8_t> dist(0, 255); //(min, max)

namespace lot49
{

//
// create implied transactions
//

void ClearVector(std::vector<uint8_t>& outPubkey)
{
    std::fill(outPubkey.begin(), outPubkey.end(), 0);
}

void SetPublicKey(std::vector<uint8_t>& outPubkey, const bls::PublicKey& inSource)
{
    inSource.Serialize(&outPubkey[0]);
}

ImpliedTransaction ImpliedTransaction::Issue(const bls::PublicKey& inReceiver, const uint16_t inFundingAmount)
{
    // cout << "Make Issue Tx" << endl;
    // issue 1:1 stored value UTXO from no previous UTXO, equivalent to mining reward (ie. no input tx)
    ImpliedTransaction tx;
    tx.mType = eIssue;

    std::vector<uint8_t> seed(bls::PrivateKey::PRIVATE_KEY_SIZE);
    std::generate_n(seed.begin(), bls::PrivateKey::PRIVATE_KEY_SIZE, [&] { return dist(rng); });
    bls::PrivateKey sk = bls::PrivateKey::FromSeed(seed.data(), seed.size());

    SetPublicKey(tx.mInputOwner1, sk.GetPublicKey());
    SetPublicKey(tx.mOutputOwner1, inReceiver);
    tx.mOutputAmount1 = inFundingAmount;
    SetPublicKey(tx.mTransactionSigner, inReceiver);
    return tx;    
}

ImpliedTransaction ImpliedTransaction::Transfer(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const uint16_t inFundingAmount)
{
    //cout << "Make Transfer Tx" << endl;
    // transfer value to 1:1 UTXO from previous 1:1 UTXO
    ImpliedTransaction tx;
    tx.mInputTxHash = inInput.GetHash();
    tx.mType = eTransfer;
    SetPublicKey(tx.mInputOwner1, inSender);
    ClearVector(tx.mInputOwner2);
    SetPublicKey(tx.mOutputOwner1, inReceiver);
    ClearVector(tx.mOutputOwner2);
    tx.mOutputAmount1 = inFundingAmount;
    tx.mOutputAmount2 = 0;
    tx.mTimeDelay = 0;
    tx.mChannelState = 0;
    ClearVector(tx.mMessageSigner);
    ClearVector(tx.mMessageHash);
    SetPublicKey(tx.mTransactionSigner, inSender);
    return tx;
}

ImpliedTransaction ImpliedTransaction::Setup(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const uint16_t inFundingAmount)
{
    //cout << "Make Setup Tx" << endl;
// fund 2:2 UTXO from previous 1:1 UTXO
    ImpliedTransaction tx(inInput);
    tx.mInputTxHash = inInput.GetHash();
    tx.mType = eSetup;
    SetPublicKey(tx.mInputOwner1, inSender);
    ClearVector(tx.mInputOwner2);
    SetPublicKey(tx.mOutputOwner1, inSender);
    SetPublicKey(tx.mOutputOwner2, inReceiver);
    tx.mOutputAmount1 = inFundingAmount;
    tx.mOutputAmount2 = 0;
    tx.mTimeDelay = 0;
    tx.mChannelState = 0;
    ClearVector(tx.mMessageSigner);
    ClearVector(tx.mMessageHash);
    SetPublicKey(tx.mTransactionSigner, inSender);
    return tx;
}

ImpliedTransaction ImpliedTransaction::Refund(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const bls::PublicKey& inSigner, const uint16_t inRefundAmount)
{
    //cout << "Make Refund Tx" << endl;
    // refund to 1:1 UTXO from previous 2:2 UTXO after delay
    ImpliedTransaction tx;
    tx.mInputTxHash = inInput.GetHash();
    tx.mType = eRefund;
    SetPublicKey(tx.mInputOwner1, inSender);
    SetPublicKey(tx.mInputOwner2, inReceiver);
    SetPublicKey(tx.mOutputOwner1, inSender);
    ClearVector(tx.mOutputOwner2);
    tx.mOutputAmount1 = inRefundAmount;
    tx.mOutputAmount2 = 0;
    tx.mTimeDelay = 7;
    tx.mChannelState = 0;
    ClearVector(tx.mMessageSigner);
    ClearVector(tx.mMessageHash);
    SetPublicKey(tx.mTransactionSigner, inSigner);
    return tx;
}

ImpliedTransaction ImpliedTransaction::UpdateAndSettle(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const bls::PublicKey& inSigner, 
    const uint16_t inSenderAmount, const uint16_t inReceiverAmount, const bls::PublicKey& inDestination, const std::vector<uint8_t>& inMessageHash)
{
    //cout << "Make UpdateAndSettle Tx" << endl;
    // update to new 2:2 UTXO or settle to two 1:1 UTXOs after delay from previous 2:2 UTXO
    ImpliedTransaction tx;
    tx.mInputTxHash = inInput.GetHash();
    tx.mType = eUpdateAndSettle;
    SetPublicKey(tx.mInputOwner1, inSender);
    SetPublicKey(tx.mInputOwner2, inReceiver);
    SetPublicKey(tx.mOutputOwner1, inSender);
    SetPublicKey(tx.mOutputOwner2, inReceiver);
    tx.mOutputAmount1 = inSenderAmount;
    tx.mOutputAmount2 = inReceiverAmount;
    tx.mTimeDelay = 7;
    tx.mChannelState = inInput.mChannelState + 1;
    SetPublicKey(tx.mMessageSigner, inDestination);
    ClearVector(tx.mMessageHash);
    SetPublicKey(tx.mTransactionSigner, inSigner);
    return tx;
}

ImpliedTransaction ImpliedTransaction::Close(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const bls::PublicKey& inSigner, 
    const uint16_t inSenderAmount, const uint16_t inReceiverAmount)
{
    //cout << "Make Close Tx" << endl;
    // refund Refund 2:2 UTXO
    ImpliedTransaction tx;
    tx.mInputTxHash = inInput.GetHash();
    tx.mType = eClose;
    SetPublicKey(tx.mInputOwner1, inSender);
    SetPublicKey(tx.mInputOwner2, inReceiver);
    SetPublicKey(tx.mOutputOwner1, inSender);
    SetPublicKey(tx.mOutputOwner2, inReceiver);
    tx.mOutputAmount1 = inSenderAmount;
    tx.mOutputAmount2 = inReceiverAmount;
    tx.mTimeDelay = 0;
    tx.mChannelState = inInput.mChannelState + 1;
    ClearVector(tx.mMessageSigner);
    ClearVector(tx.mMessageHash);
    SetPublicKey(tx.mTransactionSigner, inSigner);
    return tx;
}

// default ctor
ImpliedTransaction::ImpliedTransaction()
{
    mInputTxHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    mType = eSetup;
    mInputOwner1.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
    mInputOwner2.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
    mOutputOwner1.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
    mOutputOwner2.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
    mOutputAmount1 = 0;
    mOutputAmount2 = 0;
    mTimeDelay = 0;
    mChannelState = 0;
    mMessageSigner.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
    mMessageHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
    // not part of serialization or transaction hash
    mTransactionSigner.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
}

bool ImpliedTransaction::operator==(const ImpliedTransaction& rval) const
{
    return GetId() == rval.GetId();
}

bool ImpliedTransaction::operator<(const ImpliedTransaction& rval) const
{
    return GetId() < rval.GetId();
}

// get short transaction ID
uint32_t ImpliedTransaction::GetId() const
{
    std::vector<uint8_t> txhash = GetHash();
    uint32_t txid = bls::Util::FourBytesToInt(&txhash[0]);
    return txid;    
}

// compute hash of this transaction
std::vector<uint8_t> ImpliedTransaction::GetHash() const
{
    const std::vector<uint8_t> msg = Serialize();
    std::vector<uint8_t> message_hash(bls::BLS::MESSAGE_HASH_LEN);
    bls::Util::Hash256(&message_hash[0], reinterpret_cast<const uint8_t*>(msg.data()), msg.size());
    return message_hash;
}

// get short transaction ID of input transaction
uint32_t ImpliedTransaction::GetInputId() const
{
    std::vector<uint8_t> txhash = GetInputHash();
    uint32_t txid = bls::Util::FourBytesToInt(&txhash[0]);
    return txid;
}

// get hash of input transaction
std::vector<uint8_t> ImpliedTransaction::GetInputHash() const
{
    return mInputTxHash;
}

// compute serialization of the transaction
std::vector<uint8_t> ImpliedTransaction::Serialize() const
{
    //std::vector<uint8_t> msg(bls::PublicKey::PUBLIC_KEY_SIZE*6 + bls::BLS::MESSAGE_HASH_LEN*2 + 7);
    std::vector<uint8_t> msg(bls::PublicKey::PUBLIC_KEY_SIZE*5 + bls::BLS::MESSAGE_HASH_LEN + 7);
    auto msg_ptr = msg.begin();
    //msg_ptr = std::copy(mInputTxHash.begin(), mInputTxHash.end(), msg_ptr);
    *msg_ptr++ = static_cast<uint8_t>(mType);
    msg_ptr = std::copy(mInputOwner1.begin(), mInputOwner1.end(), msg_ptr);
    msg_ptr = std::copy(mInputOwner2.begin(), mInputOwner2.end(), msg_ptr);
    msg_ptr = std::copy(mOutputOwner1.begin(), mOutputOwner1.end(), msg_ptr);
    msg_ptr = std::copy(mOutputOwner2.begin(), mOutputOwner2.end(), msg_ptr);
    *msg_ptr = mOutputAmount1; msg_ptr+= sizeof(mOutputAmount1);
    *msg_ptr = mOutputAmount2; msg_ptr+= sizeof(mOutputAmount2);
    *msg_ptr++ = mTimeDelay;
    *msg_ptr++ = mChannelState;
    msg_ptr = std::copy(mMessageSigner.begin(), mMessageSigner.end(), msg_ptr);
    msg_ptr = std::copy(mMessageHash.begin(), mMessageHash.end(), msg_ptr);
    //std::copy(mTransactionSigner.begin(), mTransactionSigner.end(), msg_ptr);
    return msg;
}

// public key of the transaction signer
const bls::PublicKey ImpliedTransaction::GetSigner() const
{
    return bls::PublicKey::FromBytes(mTransactionSigner.data());
}

// aggregate public key of signer with public key of other signer
bool ImpliedTransaction::AddSigner(const bls::PublicKey& inSigner) 
{
    // record aggregate public key for two transaction signers; aggregate public keys in order (eg. first, second)
    const bls::PublicKey signer = GetSigner();
    bls::PublicKey input_owner0 = GetInputOwner(0);
    bls::PublicKey input_owner1 = GetInputOwner(1);
    bls::PublicKey current_signer = GetSigner();
    bool isValidSigner = current_signer == input_owner0 && inSigner == input_owner1;
    isValidSigner |= inSigner == input_owner0 && current_signer == input_owner1;
    
    // replace single public key of transaction signer with aggregate public key of both required transaction signers
    if (isValidSigner) {
        std::vector<bls::PublicKey> signers = {input_owner0, input_owner1};
        bls::PublicKey::Aggregate(signers).Serialize(mTransactionSigner.data());
    }

    assert(isValidSigner);
    return isValidSigner;
}

// return true if transaction output must be signed by public keys of two owners
bool ImpliedTransaction::IsMultisig() const
{
    if (GetType() == eIssue || GetType() == eTransfer || GetType() == eRefund) {
        return false;
    }
    return true;  
}

// get public key of output owner 0 or 1
bls::PublicKey ImpliedTransaction::GetOutputOwner(const int index) const
{
    assert(index == 0 || index == 1);
    if (index == 0) {
        return bls::PublicKey::FromBytes(mOutputOwner1.data());
    }
    return bls::PublicKey::FromBytes(mOutputOwner2.data());
}

// get total output amount for a given signing owner
uint16_t ImpliedTransaction::GetOutputAmount() const 
{
    // to spend the entire value, the new output must be signed by both owners
    return GetOutputAmount(0) + GetOutputAmount(1);
}

// get output value for owner 0 or 1
uint16_t ImpliedTransaction::GetOutputAmount(const int index) const
{
    assert(index == 0 || index == 1);
    if (index == 0) {
        return mOutputAmount1;
    }
    return mOutputAmount2;
}

// get public key of input owner 0 or 1
bls::PublicKey ImpliedTransaction::GetInputOwner(const int index) const
{
    assert(index == 0 || index == 1);
    if (index == 0) {
        return bls::PublicKey::FromBytes(mInputOwner1.data());
    }
    return bls::PublicKey::FromBytes(mInputOwner2.data());    
}

// get aggregated public key from transaction output owners
bls::PublicKey ImpliedTransaction::GetAggregateOutputOwner() const
{
    if (IsMultisig()) {
        std::vector<bls::PublicKey> owners;
        bls::PublicKey pk1 = bls::PublicKey::FromBytes(mOutputOwner1.data());
        owners.push_back(pk1);
        bls::PublicKey pk2 = bls::PublicKey::FromBytes(mOutputOwner2.data());
        owners.push_back(pk2);
        return bls::PublicKey::Aggregate(owners);
    }
    return bls::PublicKey::FromBytes(mOutputOwner1.data());
}

}; // namespace lot49