#include "MeshNode.hpp"
#include <fstream>

#pragma once

namespace lot49
{

double Distance(const std::pair<double, double>& inA, const std::pair<double, double>& inB);
void CloseLogs();
std::vector<uint8_t> HexToBytes(const std::string& hex);

std::ostream& operator<< (std::ostream& out, ETransactionType inType);
std::ostream& operator<< (std::ostream& out, EChannelState inType);
std::ostream& operator<<(std::ostream& out, const std::vector<HGID>& v);

std::ostream& operator<<(std::ostream& out, const MeshNode& n);
std::ostream& operator<<(std::ostream& out, const L49Header& i);
std::ostream& operator<<(std::ostream& out, const MeshMessage& m);

std::ofstream& TOPOLOGY();
#define _topology TOPOLOGY()

std::ofstream& LOG();
#define _log LOG()

std::ofstream& STATS();
#define _stats STATS()

};  // namespace lot49