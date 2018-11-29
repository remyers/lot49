
#include "bls.hpp"

#pragma once

namespace lot49
{
// Hashed GID
typedef uint16_t HGID;

enum ETransactionType {
    eIssue,
    eTransfer,
    eSetup,
    eRefund,
    eUpdateAndSettle,
    eClose
};

class ImpliedTransaction {
public:

// create implied transactions
static ImpliedTransaction Issue(const bls::PublicKey& inReceiver, const uint8_t inFundingAmount);
static ImpliedTransaction Transfer(const ImpliedTransaction& inInput, const bls::PublicKey& inSource, const bls::PublicKey& inReceiver, const uint8_t inFundingAmount);
static ImpliedTransaction Setup(const ImpliedTransaction& inInput, const bls::PublicKey& inSource, const bls::PublicKey& inReceiver, const uint8_t inFundingAmount);
static ImpliedTransaction Refund(const ImpliedTransaction& inInput, const bls::PublicKey& inSource, const bls::PublicKey& inReceiver, const uint8_t inRefundAmount);
static ImpliedTransaction UpdateAndSettle(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver,  
    const uint8_t inSenderAmount, const uint8_t inReceiverAmount, const bls::PublicKey& inDestination, const std::vector<uint8_t>& inMessageHash);
static ImpliedTransaction Close(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver,
    const uint8_t inSenderAmount, const uint8_t inReceiverAmount);

// compute transaction hash
std::vector<uint8_t> GetTransactionHash() const;

// compute serialization of the transaction
std::vector<uint8_t> Serialize() const;

ETransactionType GetType() const {return mType;}

// get transaction signer
// note: not part of serialization or transaction hash
std::vector<uint8_t> mTransactionSigner;

private:

ImpliedTransaction();

ETransactionType mType;
std::vector<uint8_t> mInputTxHash;
std::vector<uint8_t> mInputOwner1;
std::vector<uint8_t> mInputOwner2;
std::vector<uint8_t> mOutputOwner1;
std::vector<uint8_t> mOutputOwner2;
uint8_t mOutputAmount1;
uint8_t mOutputAmount2;
uint8_t mTimeDelay;
uint8_t mChannelState;
std::vector<uint8_t> mMessageSigner;
std::vector<uint8_t> mMessageHash;
};
}