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
    uint16_t mUnspentTokens;
    uint16_t mSpentTokens;
    uint16_t mLastNonce; // used to create unique shared 2-of-2 address
    EChannelState mState; // Setup1 -> Setup2 -> Negotiate1 -> Negotiate2 -> Receipt1 -> Receipt2 -> (Close1 -> Close2) or (back to Negotiate1)
    std::vector<uint8_t> mRefundSignature; // bls::Signature::SIGNATURE_SIZE
    std::vector<uint8_t> mPayloadHash; // bls::BLS::MESSAGE_HASH_LEN, hash of last payload - used for return receipt of payload
    std::vector<uint8_t> mWitnessHash; // bls::BLS::MESSAGE_HASH_LEN, hash of last witess - used for return receipt of witness

    bool mConfirmed; // setup tx confirmed    
};

static const uint8_t MAXRELAYS = 5;

struct L49Header
{
    bool mWitness; // witness verification?
    EChannelState mType;
    uint8_t mPrepaidTokens;
    std::vector<HGID> mRelayPath; // lot49::MAXRELAYS
    std::vector<uint8_t> mSignature; // bls::Signature::SIGNATURE_SIZE

    // used by Witness to verify a transaction chain
    std::vector<uint8_t> Serialize() const;

    // reconstruct from serialized data
    void FromBytes(const std::vector<uint8_t>& inData);
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
    std::vector<uint8_t> mPayloadData;

    // used by Witness to verify a transaction chain
    std::vector<uint8_t> Serialize() const;

    // reconstruct from serialized data
    void FromBytes(const std::vector<uint8_t>& inData);
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

    // Lookup a node from a Hashed GID
    static MeshNode &FromHGID(const HGID &inHGID);

    // Lookup a node from a public key
    static MeshNode &FromPublicKey(const bls::PublicKey& inPk);

    static void ClearRoutes();

    static HGID GetNextHop(HGID inNode, HGID inDestination);

    static bool GetNextHop(HGID inNode, HGID inDestination, HGID& outNextHop);

    static void AddGateway(HGID inNode);    

    static void PrintTopology();

    static void UpdateSimulation();

    static void WriteStats(const std::string& inLabel, const MeshMessage& inMessage);

    MeshNode();

    HGID GetHGID() const;

    // access private key
    const bls::PrivateKey GetPrivateKey() const;

    // access public key
    const bls::PublicKey GetPublicKey() const;

    // open a channel with neighbor node
    void ProposeChannel(HGID inNeighbor);

    // originate new message
    bool OriginateMessage(const HGID inDestination, const std::vector<uint8_t> &inPayload);

    // set witness node
    void SetWitnessNode(const HGID inHGID);

    // set/get correspondent node
    void SetCorrespondentNode(const HGID inHGID);
    HGID GetCorrespondentNode() const;

    // is gateway node?
    bool GetIsGateway() const;

    bool GetNearestGateway(HGID& outGatewayNode);

    friend std::ostream &operator<<(std::ostream &out, const MeshNode &n);

  private:

    // recursively find shortest route to a node
    bool FindRoute(const HGID inDestination, MeshRoute& ioRoute, std::list<HGID>& ioVisited, double& ioDistance);

    // return true if channel exists with this neighbor
    bool HasChannel(HGID inProposer, HGID inFunder) const;

    // get existing peer channel open with neighbor 
    PeerChannel& GetChannel(HGID inProposer, HGID inFunder);

    //
    bls::Signature GetAggregateSignature(const MeshMessage& inMessage, const bool isSigning);

    // 
    static std::vector<ImpliedTransaction> GetTransactions(const MeshMessage& inMessage);

    // 
    void UpdateIncentiveHeader(MeshMessage& ioMessage);

    // 
    bls::Signature SignTransaction(const ImpliedTransaction& inTransaction) const;

    // destination node signs payload 
    bls::Signature SignMessage(const std::vector<uint8_t>& inPayload) const;

    // transmit message
    void SendTransmission(const MeshMessage& inMessage);

    // receive message
    void ReceiveTransmission(const MeshMessage& inMessage);

    // check that aggregate signature is valid
    bool VerifyMessage(const MeshMessage &inMessage);

    // relay a message
    void RelayMessage(const MeshMessage& inMessage);

    // fund a channel
    void FundChannel(const MeshMessage& inMessage);

    // receive message
    void ReceiveMessage(const MeshMessage& inMessage);

    // relay delivery receipt
    void RelayDeliveryReceipt(const MeshMessage& inMessage);

    // verify the setup transaction for a payment channel with a witness node (via inGateway)
    bool ConfirmSetupTransaction(const MeshMessage& inMessage, const HGID inGateway);

    // compute serialization of the Mesh Message for Witness verification
    std::vector<uint8_t> Serialize() const;

    static std::vector<MeshNode> sNodes;

    // routes computed by routing protocol
    static std::list<MeshRoute> sRoutes;

    // state of the channel with a peer (receiving, next hop) node
    std::map<std::pair<HGID,HGID>, PeerChannel> mPeerChannels;

    HGID mWitnessNode;

    // used to create private key
    std::vector<uint8_t> mSeed;

    // gateway
    bool mIsGateway;

    // coordinates
    std::pair<double, double> mCurrentPos; // meters from origin
    std::pair<double, double> mWaypoint; // meters from origin
    HGID mCorrespondent;
    int mPausedUntil; // simulation minutes
};

}; // namespace lot49