#include "bls.hpp"
#include "MeshNode.hpp"
#include "ImpliedTransaction.hpp"

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

ImpliedTransaction ImpliedTransaction::Issue(const bls::PublicKey& inReceiver, const uint8_t inFundingAmount)
{
    // cout << "Make Issue Tx" << endl;
    // issue 1:1 stored value UTXO from no previous UTXO, equivalent to mining reward (ie. no input tx)
    ImpliedTransaction tx;
    tx.mType = eIssue;
    SetPublicKey(tx.mOutputOwner1, inReceiver);
    tx.mOutputAmount1 = inFundingAmount;
    SetPublicKey(tx.mTransactionSigner, inReceiver);
    return tx;
}

ImpliedTransaction ImpliedTransaction::Transfer(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const uint8_t inFundingAmount)
{
    //cout << "Make Transfer Tx" << endl;
    // transfer value to 1:1 UTXO from previous 1:1 UTXO
    ImpliedTransaction tx;
    tx.mInputTxHash = inInput.GetTransactionHash();
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

ImpliedTransaction ImpliedTransaction::Setup(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const uint8_t inFundingAmount)
{
    //cout << "Make Setup Tx" << endl;
// fund 2:2 UTXO from previous 1:1 UTXO
    ImpliedTransaction tx(inInput);
    tx.mInputTxHash = inInput.GetTransactionHash();
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

ImpliedTransaction ImpliedTransaction::Refund(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const bls::PublicKey& inSigner, const uint8_t inRefundAmount)
{
    //cout << "Make Refund Tx" << endl;
    // refund to 1:1 UTXO from previous 2:2 UTXO after delay
    ImpliedTransaction tx;
    tx.mInputTxHash = inInput.GetTransactionHash();
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
    const uint8_t inSenderAmount, const uint8_t inReceiverAmount, const bls::PublicKey& inDestination, const std::vector<uint8_t>& inMessageHash)
{
    //cout << "Make UpdateAndSettle Tx" << endl;
    // update to new 2:2 UTXO or settle to two 1:1 UTXOs after delay from previous 2:2 UTXO
    ImpliedTransaction tx;
    tx.mInputTxHash = inInput.GetTransactionHash();
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
    const uint8_t inSenderAmount, const uint8_t inReceiverAmount)
{
    //cout << "Make Close Tx" << endl;
    // refund Refund 2:2 UTXO
    ImpliedTransaction tx;
    tx.mInputTxHash = inInput.GetTransactionHash();
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

// compute transaction hash
std::vector<uint8_t> ImpliedTransaction::GetTransactionHash() const
{
    const std::vector<uint8_t> msg = Serialize();
    vector<uint8_t> message_hash(bls::BLS::MESSAGE_HASH_LEN);
    bls::Util::Hash256(&message_hash[0], reinterpret_cast<const uint8_t*>(msg.data()), msg.size());
    return message_hash;
}

// compute serialization of the transaction
std::vector<uint8_t> ImpliedTransaction::Serialize() const
{
    std::vector<uint8_t> msg(bls::PublicKey::PUBLIC_KEY_SIZE*6 + bls::BLS::MESSAGE_HASH_LEN*2 + 5);
    auto msg_ptr = msg.begin();
    msg_ptr = std::copy(mInputTxHash.begin(), mInputTxHash.end(), msg_ptr);
    *msg_ptr++ = static_cast<uint8_t>(mType);
    msg_ptr = std::copy(mInputOwner1.begin(), mInputOwner1.end(), msg_ptr);
    msg_ptr = std::copy(mInputOwner2.begin(), mInputOwner2.end(), msg_ptr);
    msg_ptr = std::copy(mOutputOwner1.begin(), mOutputOwner1.end(), msg_ptr);
    msg_ptr = std::copy(mOutputOwner2.begin(), mOutputOwner2.end(), msg_ptr);
    *msg_ptr++ = mOutputAmount1;
    *msg_ptr++ = mOutputAmount2;
    *msg_ptr++ = mTimeDelay;
    *msg_ptr++ = mChannelState;
    msg_ptr = std::copy(mMessageSigner.begin(), mMessageSigner.end(), msg_ptr);
    msg_ptr = std::copy(mMessageHash.begin(), mMessageHash.end(), msg_ptr);
    std::copy(mTransactionSigner.begin(), mTransactionSigner.end(), msg_ptr);
    return msg;
}

// public key of the transaction signer
const bls::PublicKey ImpliedTransaction::GetSigner() const
{
    return bls::PublicKey::FromBytes(mTransactionSigner.data());
}

}; // namespace lot49