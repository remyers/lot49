#include "bls.hpp"
#include "MeshNode.hpp"
#include "ImpliedTransaction.hpp"

namespace lot49
{

//
// create implied transactions
//

ImpliedTransaction ImpliedTransaction::Setup(bls::PublicKey inSource, bls::PublicKey inReceiver, uint8_t inFundingAmount)
{
    ImpliedTransaction tx;
}

ImpliedTransaction ImpliedTransaction::Refund(bls::PublicKey inSource, bls::PublicKey inReceiver, uint8_t inRefundAmount)
{
}

ImpliedTransaction ImpliedTransaction::UpdateAndSettle(bls::PublicKey inSender, bls::PublicKey inReceiver, uint16_t inChannelState, uint8_t inSenderAmount, uint8_t inReceiverAmount, bls::PublicKey inDestinationOwner, std::vector<uint8_t>& inMessageHash)
{
    
}

ImpliedTransaction ImpliedTransaction::Close(bls::PublicKey inSender, bls::PublicKey inReceiver, uint16_t inChannelState, uint8_t inSenderAmount, uint8_t inReceiverAmount)
{
}

// default ctor
ImpliedTransaction::ImpliedTransaction()
{
    mType = eSetup;
    mInputOwner1.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
    mInputOwner2.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
    mOutputOwner1.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
    mOutputOwner2.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
    mOutputAmount1 = 0;
    mOutputAmount2 = 0;
    mTimeDelay = 0;
    mChannelState = eSetup1;
    mMessageSigner.resize(bls::PublicKey::PUBLIC_KEY_SIZE, 0);
    mMessageHash.resize(bls::BLS::MESSAGE_HASH_LEN, 0);
}

// compute serialization of the transaction
std::vector<uint8_t> ImpliedTransaction::Serialize() const
{
    std::vector<uint8_t> msg(bls::PublicKey::PUBLIC_KEY_SIZE*5 + bls::BLS::MESSAGE_HASH_LEN + 5);
    auto msg_ptr = msg.begin();
    *msg_ptr++ = static_cast<uint8_t>(mType);
    msg_ptr = std::copy(mInputOwner1.begin(), mInputOwner1.end(), msg_ptr);
    msg_ptr = std::copy(mInputOwner2.begin(), mInputOwner2.end(), msg_ptr);
    msg_ptr = std::copy(mOutputOwner1.begin(), mOutputOwner1.end(), msg_ptr);
    msg_ptr = std::copy(mOutputOwner2.begin(), mOutputOwner2.end(), msg_ptr);
    *msg_ptr++ = mOutputAmount1;
    *msg_ptr++ = mOutputAmount2;
    *msg_ptr++ = mTimeDelay;
    *msg_ptr++ = static_cast<uint8_t>(mChannelState);
    msg_ptr = std::copy(mMessageSigner.begin(), mMessageSigner.end(), msg_ptr);
    std::copy(mMessageHash.begin(), mMessageHash.end(), msg_ptr);
    return msg;
}

}; // namespace lot49