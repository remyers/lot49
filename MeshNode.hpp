using namespace std;

#include "bls.hpp"
#include "ImpliedTransaction.hpp"

#include <list>
#include <memory>

#pragma once

namespace lot49
{

// Hashed GID
typedef uint16_t HGID;

// Mesh route
typedef std::vector<HGID> MeshRoute;

//
// incentive headers
//

enum EChannelState 
{
    eSetup1,
    eSetup2,
    eNegotiate1,
    eNegotiate2,
    eNegotiate3,
    eReceipt1,
    eReceipt2,
    eClose1,
    eClose2
};

struct PeerChannel
{
    HGID mFundingPeer;
    HGID mProposingPeer;
    uint8_t mUnspentTokens;
    uint8_t mSpentTokens;
    uint16_t mLastNonce; // used to create unique shared 2-of-2 address
    EChannelState mState; // Setup1 -> Setup2 -> Negotiate1 -> Negotiate2 -> Receipt1 -> Receipt2 -> (Close1 -> Close2) or (back to Negotiate1)
    std::vector<uint8_t> mRefundSignature; // bls::Signature::SIGNATURE_SIZE
    std::vector<uint8_t> mPayloadHash; // bls::BLS::MESSAGE_HASH_LEN, hash of last payload - used for return receipt

    bool mConfirmed; // setup tx confirmed
};

static const uint8_t MAXRELAYS = 8;

struct L49Header
{
    EChannelState mType;
    uint8_t mPrepaidTokens;
    std::vector<HGID> mRelayPath; // lot49::MAXRELAYS
    std::vector<uint8_t> mSignature; // bls::Signature::SIGNATURE_SIZE
};

//
// Mesh Message
//

struct MeshMessage
{
    HGID mSender;
    HGID mReceiver;
    HGID mSource;
    HGID mDestination;
    L49Header mIncentive;

    // payload, max 236 bytes
    std::string mPayloadData;
};

enum EEventType
{
    eOriginate,
    eTransmit,
    eReceive
};

struct MessageEvent
{
    EEventType mEvent;
    MeshMessage mMessage;
    uint32_t mTimestamp;
    uint16_t mNonce; // recreate payment address using mMessage.mSenderHGID + mMessage.mReceiverHGID + mNonce
};

class MeshNode
{
  public:

    // create mesh nodes
    static void CreateNodes(const int inCount);

    static MeshNode &FromIndex(const int inIndex);

    // configure topology
    static void AddRoute(MeshRoute inRoute);
    
    static bool HasNeighbor(HGID inNode, HGID inNeighbor);

    // Lookup or construct a node from a Hashed GID
    static MeshNode &FromHGID(const HGID &inHGID);

    static HGID GetNextHop(HGID inNode, HGID inDestination);

    HGID GetHGID() const;

    // access private key
    const bls::PrivateKey GetPrivateKey() const;

    // access public key
    const bls::PublicKey GetPublicKey() const;

    // open a channel with neighbor node
    void ProposeChannel(HGID inNeighbor);

    // originate new message
    void OriginateMessage(const HGID inDestination, const std::string &inPayload);

    friend std::ostream &operator<<(std::ostream &out, const MeshNode &n);

  private:
    MeshNode();

    // return true if channel exists with this neighbor
    bool HasChannel(HGID inNeighbor) const;

    // get existing peer channel open with neighbor 
    PeerChannel& GetChannel(HGID inNeighbor);

    //
    bls::Signature GetAggregateSignature(const MeshMessage& inMessage, const bool isUpdatingSignature);

    // 
    std::vector<ImpliedTransaction> GetTransactions(const MeshMessage& inMessage);

    // 
    void UpdateIncentiveHeader(MeshMessage& ioMessage);

    // 
    bls::Signature SignTransaction(const ImpliedTransaction& inTransaction) const;

    // destination node signs payload 
    bls::Signature SignMessage(const std::string& inPayload) const;

    // transmit message
    void SendTransmission(const MeshMessage& inMessage);

    // receive message
    void ReceiveTransmission(const MeshMessage& inMessage);

    // check that aggregate signature is valid
    bool VerifyMessage(const MeshMessage &inMessage);

    // relay a message
    bool RelayMessage(const MeshMessage& inMessage);

    // fund a channel
    bool FundChannel(const MeshMessage& inMessage);

    // receive message
    bool ReceiveMessage(const MeshMessage& inMessage);

    // receive delivery receipt
    bool RelayDeliveryReceipt(const MeshMessage& inMessage);

    static vector<MeshNode> sNodes;

    // routes computed by routing protocol
    static std::list<MeshRoute> sRoutes;

    std::list<MessageEvent> mMessageEvents;
    std::list<PeerChannel> mPeerChannels;

    // used to create private key
    std::vector<uint8_t> mSeed;
};

}; // namespace lot49