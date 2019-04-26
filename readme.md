# Lot49 

This project exists to explore and simulate ideas for a [Layer 2](https://en.wikipedia.org/wiki/Lightning_Network) based payment protocol optimized for incentivizing data delivery in an adhoc mesh network.

## Protocol Summary

* Sender nodes source route message data (think SMS, not TCP/IP) to Destination nodes.
* Sender nodes include a payment if routing via Relay nodes or internet Gateway nodes.
* Nodes that relay data for a Sender collect a payment only if the Destination node receives the message data.
* Payment transactions are verified and settled on a decentralized ledger using online Witness nodes.

![Lot49 Network Overview](./doc/lot49_network_overview.svg?raw=true "Lot49 Network Overview")

## Simulated Radio Specifications

* Message Size - up to 236 bytes per message
* Data Rate - 5 messages per minute
* Range - 1600 meters

## Fixed Simulation Parameters

* Simulation Rate - 1 update per minute
* Message Size - size of payload messages (excluding headers) (50 bytes)
* Area - size of mesh area (5 x 5 km)
* ID size - size of node ID (2 bytes)
* Incentive Header Overhead - 4 bytes + relay path + signature (see below)
* Message Hash - size of message hash (32 bytes)
* Relay Incentive - how likely a node is to relay without incentives (0%, incentives only)
* Send Incentive - how likely a node is to send a message with an incentive (100%)
* Broadcast Success - per broadcast chance of delivery success for message sent from node A * to B based on distance and local node density (100% if in radio range, otherwise won’t send)
* Correspondents - number of other nodes to communicate with (1 other node for each node in the simulation, always next node numerically)
* Duration - how long to run the simulation with the same set of peers and correspondents. (up to 120 minutes)
* Channel Size - default channel starting number of tokens (2000 tokens)
* Relay Cost - default number of tokens taken per relay (1 token)
* Prepaid Cost - default number of tokens prepaid to send a message (# of hops + 1 token)
* Setup Failure - chance a channel setup transaction is not valid, ie. double spent (0%)
* Pause Time - how long will nodes pause once they have reached their random waypoint (5 minutes).
* Radio Range - successful radio broadcast distance (1600 meters)
* Count Single Hop Correspondents: No, only when originated message is sent via a relay.
* Speed - speed that nodes move (85 meters per minute - walking pace)

## Variable simulation parameters

* Density - average number of nodes per square meter (0.5, 1, 2 or 3 node per sq km)
* Signature Size - size of aggregate signature (32, 48, 64 or 96 bytes)
* Gateways - percent of nodes acting as gateways
* Witness Channel Setup - confirm new channel setup transactions immediately vs. deferred confirmation

![Lot49 Simulation](./doc/lot49_simulation.svg?raw=true "Lot49 Simulation")

## FAQ

Q: Why not use the current Bitcoin network to settle payment transactions?

Currently Bitcoin uses ECDSA signatures that can not be non-interactively aggregated like [BLS signatures](https://crypto.stanford.edu/~dabo/pubs/papers/aggreg.pdf) can. Bitcoin also does not support [SIGHASH_NOINPUT](https://github.com/bitcoin/bips/blob/master/bip-0118.mediawiki) which is required for the [eltoo](https://blockstream.com/eltoo.pdf) update scheme.

Q: Are there any alternatives to using BLS signatures and non-interactive signature aggregation?

A: It may be possible to use [Schnorr based Multisignatures](https://eprint.iacr.org/2018/068.pdf) to accomplish nearly the same level of overhead reduction as the more general BLS signature aggregation.

Q: Are there any alternatives to using the eltoo update scheme?

A: The current Lightning Network revocation system could also be made to work, but it would be more complicated and require more communication overhead.



