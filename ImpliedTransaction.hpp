
#include "bls.hpp"

#pragma once

namespace lot49
{
// Hashed GID
typedef uint16_t HGID;

enum ETransactionType {
    eSetup,
    eRefund,
    eUpdateAndSettle,
    eClose
};

class ImpliedTransaction {
public:

// create implied transactions
static ImpliedTransaction Setup(bls::PublicKey inSource, bls::PublicKey inReceiver, uint8_t inFundingAmount);
static ImpliedTransaction Refund(bls::PublicKey inSource, bls::PublicKey inReceiver, uint8_t inRefundAmount);
static ImpliedTransaction UpdateAndSettle(bls::PublicKey inSender, bls::PublicKey inReceiver, uint16_t inNonce, uint8_t inSenderAmount, uint8_t inReceiverAmount, bls::PublicKey inDestinationOwner, std::vector<uint8_t>& inMessageHash);
static ImpliedTransaction Close(bls::PublicKey inSender, bls::PublicKey inReceiver, uint16_t inChannelState, uint8_t inSenderAmount, uint8_t inReceiverAmount);

// compute serialization of the transaction
std::vector<uint8_t> Serialize() const;

ETransactionType GetType() const {return mType;}

private:

ImpliedTransaction();

ETransactionType mType;
std::vector<uint8_t> mInputOwner1;
std::vector<uint8_t> mInputOwner2;
std::vector<uint8_t> mOutputOwner1;
std::vector<uint8_t> mOutputOwner2;
uint8_t mOutputAmount1;
uint8_t mOutputAmount2;
uint8_t mTimeDelay;
EChannelState mChannelState;
std::vector<uint8_t> mMessageSigner;
std::vector<uint8_t> mMessageHash;
};
}