
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
static ImpliedTransaction Refund(const ImpliedTransaction& inInput, const bls::PublicKey& inSource, const bls::PublicKey& inReceiver, const bls::PublicKey& inSigner, const uint8_t inRefundAmount);
static ImpliedTransaction UpdateAndSettle(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const bls::PublicKey& inSigner, 
    const uint8_t inSenderAmount, const uint8_t inReceiverAmount, const bls::PublicKey& inDestination, const std::vector<uint8_t>& inMessageHash);
static ImpliedTransaction Close(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const bls::PublicKey& inSigner, 
    const uint8_t inSenderAmount, const uint8_t inReceiverAmount);

// compute transaction hash
std::vector<uint8_t> GetTransactionHash() const;

// compute serialization of the transaction
std::vector<uint8_t> Serialize() const;

ETransactionType GetType() const {return mType;}

// public key of the transaction signer
const bls::PublicKey GetSigner() const;

private:

ImpliedTransaction();

ETransactionType mType;
// TODO: input transaction hash should correctly point to specific tx or be empty if this tx is signed with NOINPUT signature flag
std::vector<uint8_t> mInputTxHash; 
// TODO: public keys should be generated from an extended public key + nonce
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

// public key of the transaction signer 
// NOTE: also part of the serialization and transaction hash to avoid two signers signing identical transactions
std::vector<uint8_t> mTransactionSigner;
};
}