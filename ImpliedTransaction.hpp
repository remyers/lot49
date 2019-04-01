
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
    static ImpliedTransaction Issue(const bls::PublicKey& inReceiver, const uint16_t inFundingAmount);
    static ImpliedTransaction Issue(const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, const uint16_t inFundingAmount);
    static ImpliedTransaction Transfer(const ImpliedTransaction& inInput, const bls::PublicKey& inSource, const bls::PublicKey& inReceiver, 
        const uint16_t inFundingAmount);
    static ImpliedTransaction Setup(const ImpliedTransaction& inInput, const bls::PublicKey& inSource, const bls::PublicKey& inReceiver, 
        const uint16_t inFundingAmount);
    static ImpliedTransaction Refund(const ImpliedTransaction& inInput, const bls::PublicKey& inSource, const bls::PublicKey& inReceiver, 
        const bls::PublicKey& inSigner, const uint16_t inRefundAmount);
    static ImpliedTransaction UpdateAndSettle(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, 
        const bls::PublicKey& inSigner, const uint16_t inSenderAmount, const uint16_t inReceiverAmount, const bls::PublicKey& inDestination, 
        const std::vector<uint8_t>& inMessageHash);
    static ImpliedTransaction Close(const ImpliedTransaction& inInput, const bls::PublicKey& inSender, const bls::PublicKey& inReceiver, 
        const bls::PublicKey& inSigner, const uint16_t inSenderAmount, const uint16_t inReceiverAmount);

    // default ctor
    ImpliedTransaction();

    // operators to sort/map transactions
    bool operator==(const ImpliedTransaction& rval) const;
    bool operator<(const ImpliedTransaction& rval) const;

    // get short transaction ID
    uint32_t GetId() const;

    // compute hash of this transaction
    std::vector<uint8_t> GetHash() const;

    // get short transaction ID of input transaction
    uint32_t GetInputId() const;

    // get hash of input transaction
    std::vector<uint8_t> GetInputHash() const;

    // compute serialization of the transaction
    std::vector<uint8_t> Serialize() const;

    ETransactionType GetType() const { return mType; }

    // public key of the transaction signer
    const bls::PublicKey GetSigner() const;

    // aggregate public key of signer with public key of other signer
    bool AddSigner(const bls::PublicKey& inSigner);

    // return true if transaction output must be signed by public keys of two owners
    bool IsMultisig() const;

    // get public key of output owner 0 or 1
    bls::PublicKey GetOutputOwner(const int index) const;

    // get total output amount
    uint16_t GetOutputAmount() const;

    // get output value for owner 0 or 1
    uint16_t GetOutputAmount(const int index) const;

    // get public key of input owner 0 or 1
    bls::PublicKey GetInputOwner(const int index) const;

    // get public key aggregated from all transaction output owner public keys
    bls::PublicKey GetAggregateOutputOwner() const;

    private:

    ETransactionType mType;
    // TODO: input transaction hash should correctly point to specific tx or be empty if this tx is signed with NOINPUT signature flag
    std::vector<uint8_t> mInputTxHash; 
    // TODO: public keys should be generated from an extended public key + nonce
    std::vector<uint8_t> mInputOwner1;
    std::vector<uint8_t> mInputOwner2;
    std::vector<uint8_t> mOutputOwner1;
    std::vector<uint8_t> mOutputOwner2;
    uint16_t mOutputAmount1;
    uint16_t mOutputAmount2;
    uint8_t mTimeDelay;
    uint8_t mChannelState;
    std::vector<uint8_t> mMessageSigner; // public key of destination node that must sign the message hash
    std::vector<uint8_t> mMessageHash;

    // public key of the transaction signer 
    // NOTE: also part of the serialization and transaction hash to avoid two signers signing identical transactions
    std::vector<uint8_t> mTransactionSigner;
};
}