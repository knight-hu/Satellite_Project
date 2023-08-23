#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include <string>
#include <fstream>
#include <string>
#include <cmath>
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-nix-vector-helper.h"
#include "ns3/ipv4-nix-vector-routing.h"

#define PI 3.1415926535897
using namespace ns3;

void Progress2 (NodeContainer nodes) {
	Ptr<Node> from = nodes.Get(10);
	Ptr<Node> to = nodes.Get(43);
	Ptr<Ipv4NixVectorRouting> rp = from->GetObject<Ipv4NixVectorRouting> ();
	std::vector< Ptr<Node> > path_temp = rp->GetShortPath(from, to);
	std::cout<<"---5---";
	for(uint i = 0; i < path_temp.size(); i++){
		std::cout<<path_temp[i]->GetId()<<" " ;
	}
	std::cout<<std::endl;
}

//void
//Progress3 (Ptr<Node> node) {
//	Ptr<LEOSatelliteMobilityModel> position = node->GetObject<LEOSatelliteMobilityModel> ();
//	position->m_helper.m_pos.isRightSatelliteConnection = false;
//}

// 根据ISL的状态进行快照
void newSnapshotWay(NodeContainer nodes, double simulatorLastTime, double T){
	std::set<double> s;
	for (NodeContainer::Iterator j = nodes.Begin ();j != nodes.End (); ++j){
	      Ptr<Node> object = *j;
	      Ptr<LEOSatelliteMobilityModel> mobility = object->GetObject<LEOSatelliteMobilityModel> ();
	      double enterNorthPoleTime = mobility->m_helper.getEnterNorthPoleTime();
	      while (enterNorthPoleTime <= simulatorLastTime) {
	    	  s.insert(enterNorthPoleTime);
	    	  enterNorthPoleTime = enterNorthPoleTime + T;
	      }
	      double enterSorthPoleTime = mobility->m_helper.getEnterSorthPoleTime();
	      while (enterSorthPoleTime <= simulatorLastTime) {
	    	  s.insert(enterSorthPoleTime);
	    	  enterSorthPoleTime = enterSorthPoleTime + T;
	      }
	      double leaveNorthPoleTime = mobility->m_helper.getLeaveNorthPoleTime();
		  while (leaveNorthPoleTime <= simulatorLastTime) {
			  s.insert(leaveNorthPoleTime);
			  leaveNorthPoleTime = leaveNorthPoleTime + T;
		  }
		  double leaveSorthPoleTime = mobility->m_helper.getLeaveSorthPoleTime();
		  while (leaveSorthPoleTime <= simulatorLastTime) {
			  s.insert(leaveSorthPoleTime);
			  leaveSorthPoleTime = leaveSorthPoleTime + T;
		  }
	}
	std::cout<<"newSnapshotWay: set.size= "<< s.size() << std::endl;
	std::set<double>::iterator iter;
    for(iter = s.begin() ; iter != s.end() ; ++iter){
         std::cout<<*iter<<" ";
     }
    std::cout<<std::endl;
	for (NodeContainer::Iterator j = nodes.Begin ();j != nodes.End (); ++j){
	      Ptr<Node> object = *j;
	      Ptr<LEOSatelliteMobilityModel> mobility = object->GetObject<LEOSatelliteMobilityModel> ();
	      for(iter = s.begin() ; iter != s.end() ; ++iter){
	 		  Simulator::Schedule (Seconds (*iter+0.01), &LEOSatelliteMobilityModel::UpdatePosition, mobility);
	       }
	}
//	Simulator::Schedule (Seconds (186.25), Progress3,nodes.Get(1));
//	Simulator::Schedule (Seconds (186.26), Progress2,nodes);
}

// 路径变成父节点数组，为了调用nix内部的api
std::vector< Ptr<Node> >
buildParentVector(std::vector< Ptr<Node> > & path){
	std::vector< Ptr<Node> > parentVector;
	// reset the parent vector
	parentVector.clear ();
	uint32_t numberOfNodes = NodeList::GetNNodes ();
    parentVector.reserve (sizeof (Ptr<Node>)*numberOfNodes);
    parentVector.insert (parentVector.begin (), sizeof (Ptr<Node>)*numberOfNodes, 0); // initialize to 0

	int size = path.size();
	if(size <= 0 ) return parentVector;
	Ptr<Node> pre = path.at(0);
	parentVector.at (pre->GetId ()) = pre;
	Ptr<Node> current;
	for(int i = 1; i < size; i++) {
		current = path.at(i);
		parentVector.at (current->GetId ()) = pre;
		pre = current;
	}
	return parentVector;
}

void Progress (Ipv4Address destinationIp, Ptr<Node> source, Ptr<Node> dest, double timeInterval, double clientEndTime) {
	double currentTime = Simulator::Now ().GetSeconds();
	if(currentTime > clientEndTime) return;
    std::cout << "时间: "<< Simulator::Now ().GetMilliSeconds() << "  Progress" << std::endl;
	Ptr<LEOSatelliteMobilityModel> source_position = source->GetObject<LEOSatelliteMobilityModel> ();
	std::vector< Ptr<Node> > newPath = source_position->m_helper.SWSMainDeal(dest); // SWS算法

	std::cout<< "new path: ";
	for (uint i=0;i<newPath.size();i++){
		std::cout<<newPath[i]->GetId()<<" ";
	}
	std::cout<<std::endl;
	std::vector< Ptr<Node> > parentVector = buildParentVector(newPath);//父节点数组
	Ptr<Ipv4NixVectorRouting> rp = source->GetObject<Ipv4NixVectorRouting> ();
	Simulator::ScheduleNow (&Ipv4NixVectorRouting::UpdateNixVectorInCache, rp ,destinationIp, parentVector, source, dest);//规定nix路由走分段路由
    Simulator::Schedule (Seconds (timeInterval), Progress, destinationIp, source, dest, timeInterval, clientEndTime);
}

void setFaultSatellite (Ptr<Node> node, bool flag) {
    std::cout << "时间: "<< Simulator::Now ().GetMilliSeconds() << "  setFaultSatellite" << std::endl;
	Ptr<LEOSatelliteMobilityModel> node_position = node->GetObject<LEOSatelliteMobilityModel> ();
	node_position->m_helper.setIsFault(flag); // @suppress("Invalid arguments")
}

int main (int argc, char *argv[])
{
  // 设置接口输入
  CommandLine cmd;
  cmd.Parse (argc, argv);
  Time::SetResolution (Time::NS);

  //必须有下面2个语句，才会打印udp的信息，默认是info级别的
  LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);

  // 当前开始时间
  time_t t = time(0);
  char tmp[64];
  strftime( tmp, sizeof(tmp), "%Y/%m/%d %X",localtime(&t) );
  puts( tmp );

  // 星座参数
  const double ALTITUDE = 780;//卫星海拔
  const double INCLINATION = 86.4;//倾角
  const double planeNum = 20;
  const int satelliteInOnePlane = 50;
  // Format: nodenumber, longitude（升交点赤经，角度制）, alpha(距离近地点的角度，角度制，因为是圆轨道，没有近地点，这里假设近地点是地球正面的赤道) and plane(轨道编号).
  // double iridiumConstellation[6][11][4] = {{{0,0,0.0,0},{1,0,32.73,0},{2,0,65.45,0},{3,0,98.18,0},{4,0,130.91,0},{5,0,163.64,0},{6,0,196.36,0},{7,0,229.09,0},{8,0,261.82,0},{9,0,294.55,0},{10,0,327.27,0}},{{11,30,0.0,1},{12,30,32.73,1},{13,30,65.45,1},{14,30,98.18,1},{15,30,130.91,1},{16,30,163.64,1},{17,30,196.36,1},{18,30,229.09,1},{19,30,261.82,1},{20,30,294.55,1},{21,30,327.27,1}},{{22,60,0.0,2},{23,60,32.73,2},{24,60,65.45,2},{25,60,98.18,2},{26,60,130.91,2},{27,60,163.64,2},{28,60,196.36,2},{29,60,229.09,2},{30,60,261.82,2},{31,60,294.55,2},{32,60,327.27,2}},{{33,90,0.0,3},{34,90,32.73,3},{35,90,65.45,3},{36,90,98.18,3},{37,90,130.91,3},{38,90,163.64,3},{39,90,196.36,3},{40,90,229.09,3},{41,90,261.82,3},{42,90,294.55,3},{43,90,327.27,3}},{{44,120,0.0,4},{45,120,32.73,4},{46,120,65.45,4},{47,120,98.18,4},{48,120,130.91,4},{49,120,163.64,4},{50,120,196.36,4},{51,120,229.09,4},{52,120,261.82,4},{53,120,294.55,4},{54,120,327.27,4}},{{55,150,0.0,5},{56,150,32.73,5},{57,150,65.45,5},{58,150,98.18,5},{59,150,130.91,5},{60,150,163.64,5},{61,150,196.36,5},{62,150,229.09,5},{63,150,261.82,5},{64,150,294.55,5},{65,150,327.27,5}}};
  // double iridiumConstellation[6][11][4] = {{{0,0,0.0,0},{1,0,32.73,0},{2,0,65.45,0},{3,0,98.18,0},{4,0,130.91,0},{5,0,163.64,0},{6,0,196.36,0},{7,0,229.09,0},{8,0,261.82,0},{9,0,294.55,0},{10,0,327.27,0}},{{11,30,16.36,1},{12,30,49.09,1},{13,30,81.82,1},{14,30,114.55,1},{15,30,147.27,1},{16,30,180.0,1},{17,30,212.73,1},{18,30,245.45,1},{19,30,278.18,1},{20,30,310.91,1},{21,30,343.64,1}},{{22,60,0.0,2},{23,60,32.73,2},{24,60,65.45,2},{25,60,98.18,2},{26,60,130.91,2},{27,60,163.64,2},{28,60,196.36,2},{29,60,229.09,2},{30,60,261.82,2},{31,60,294.55,2},{32,60,327.27,2}},{{33,90,16.36,3},{34,90,49.09,3},{35,90,81.82,3},{36,90,114.55,3},{37,90,147.27,3},{38,90,180.0,3},{39,90,212.73,3},{40,90,245.45,3},{41,90,278.18,3},{42,90,310.91,3},{43,90,343.64,3}},{{44,120,0.0,4},{45,120,32.73,4},{46,120,65.45,4},{47,120,98.18,4},{48,120,130.91,4},{49,120,163.64,4},{50,120,196.36,4},{51,120,229.09,4},{52,120,261.82,4},{53,120,294.55,4},{54,120,327.27,4}},{{55,150,16.36,5},{56,150,49.09,5},{57,150,81.82,5},{58,150,114.55,5},{59,150,147.27,5},{60,150,180.0,5},{61,150,212.73,5},{62,150,245.45,5},{63,150,278.18,5},{64,150,310.91,5},{65,150,343.64,5}}};
  double iridiumConstellation[20][50][4] = {{{0,0,0.0,0},{1,0,7.2,0},{2,0,14.4,0},{3,0,21.6,0},{4,0,28.8,0},{5,0,36.0,0},{6,0,43.2,0},{7,0,50.4,0},{8,0,57.6,0},{9,0,64.8,0},{10,0,72.0,0},{11,0,79.2,0},{12,0,86.4,0},{13,0,93.6,0},{14,0,100.8,0},{15,0,108.0,0},{16,0,115.2,0},{17,0,122.4,0},{18,0,129.6,0},{19,0,136.8,0},{20,0,144.0,0},{21,0,151.2,0},{22,0,158.4,0},{23,0,165.6,0},{24,0,172.8,0},{25,0,180.0,0},{26,0,187.2,0},{27,0,194.4,0},{28,0,201.6,0},{29,0,208.8,0},{30,0,216.0,0},{31,0,223.2,0},{32,0,230.4,0},{33,0,237.6,0},{34,0,244.8,0},{35,0,252.0,0},{36,0,259.2,0},{37,0,266.4,0},{38,0,273.6,0},{39,0,280.8,0},{40,0,288.0,0},{41,0,295.2,0},{42,0,302.4,0},{43,0,309.6,0},{44,0,316.8,0},{45,0,324.0,0},{46,0,331.2,0},{47,0,338.4,0},{48,0,345.6,0},{49,0,352.8,0}},{{50,9,3.6,1},{51,9,10.8,1},{52,9,18.0,1},{53,9,25.2,1},{54,9,32.4,1},{55,9,39.6,1},{56,9,46.8,1},{57,9,54.0,1},{58,9,61.2,1},{59,9,68.4,1},{60,9,75.6,1},{61,9,82.8,1},{62,9,90.0,1},{63,9,97.2,1},{64,9,104.4,1},{65,9,111.6,1},{66,9,118.8,1},{67,9,126.0,1},{68,9,133.2,1},{69,9,140.4,1},{70,9,147.6,1},{71,9,154.8,1},{72,9,162.0,1},{73,9,169.2,1},{74,9,176.4,1},{75,9,183.6,1},{76,9,190.8,1},{77,9,198.0,1},{78,9,205.2,1},{79,9,212.4,1},{80,9,219.6,1},{81,9,226.8,1},{82,9,234.0,1},{83,9,241.2,1},{84,9,248.4,1},{85,9,255.6,1},{86,9,262.8,1},{87,9,270.0,1},{88,9,277.2,1},{89,9,284.4,1},{90,9,291.6,1},{91,9,298.8,1},{92,9,306.0,1},{93,9,313.2,1},{94,9,320.4,1},{95,9,327.6,1},{96,9,334.8,1},{97,9,342.0,1},{98,9,349.2,1},{99,9,356.4,1}},{{100,18,0.0,2},{101,18,7.2,2},{102,18,14.4,2},{103,18,21.6,2},{104,18,28.8,2},{105,18,36.0,2},{106,18,43.2,2},{107,18,50.4,2},{108,18,57.6,2},{109,18,64.8,2},{110,18,72.0,2},{111,18,79.2,2},{112,18,86.4,2},{113,18,93.6,2},{114,18,100.8,2},{115,18,108.0,2},{116,18,115.2,2},{117,18,122.4,2},{118,18,129.6,2},{119,18,136.8,2},{120,18,144.0,2},{121,18,151.2,2},{122,18,158.4,2},{123,18,165.6,2},{124,18,172.8,2},{125,18,180.0,2},{126,18,187.2,2},{127,18,194.4,2},{128,18,201.6,2},{129,18,208.8,2},{130,18,216.0,2},{131,18,223.2,2},{132,18,230.4,2},{133,18,237.6,2},{134,18,244.8,2},{135,18,252.0,2},{136,18,259.2,2},{137,18,266.4,2},{138,18,273.6,2},{139,18,280.8,2},{140,18,288.0,2},{141,18,295.2,2},{142,18,302.4,2},{143,18,309.6,2},{144,18,316.8,2},{145,18,324.0,2},{146,18,331.2,2},{147,18,338.4,2},{148,18,345.6,2},{149,18,352.8,2}},{{150,27,3.6,3},{151,27,10.8,3},{152,27,18.0,3},{153,27,25.2,3},{154,27,32.4,3},{155,27,39.6,3},{156,27,46.8,3},{157,27,54.0,3},{158,27,61.2,3},{159,27,68.4,3},{160,27,75.6,3},{161,27,82.8,3},{162,27,90.0,3},{163,27,97.2,3},{164,27,104.4,3},{165,27,111.6,3},{166,27,118.8,3},{167,27,126.0,3},{168,27,133.2,3},{169,27,140.4,3},{170,27,147.6,3},{171,27,154.8,3},{172,27,162.0,3},{173,27,169.2,3},{174,27,176.4,3},{175,27,183.6,3},{176,27,190.8,3},{177,27,198.0,3},{178,27,205.2,3},{179,27,212.4,3},{180,27,219.6,3},{181,27,226.8,3},{182,27,234.0,3},{183,27,241.2,3},{184,27,248.4,3},{185,27,255.6,3},{186,27,262.8,3},{187,27,270.0,3},{188,27,277.2,3},{189,27,284.4,3},{190,27,291.6,3},{191,27,298.8,3},{192,27,306.0,3},{193,27,313.2,3},{194,27,320.4,3},{195,27,327.6,3},{196,27,334.8,3},{197,27,342.0,3},{198,27,349.2,3},{199,27,356.4,3}},{{200,36,0.0,4},{201,36,7.2,4},{202,36,14.4,4},{203,36,21.6,4},{204,36,28.8,4},{205,36,36.0,4},{206,36,43.2,4},{207,36,50.4,4},{208,36,57.6,4},{209,36,64.8,4},{210,36,72.0,4},{211,36,79.2,4},{212,36,86.4,4},{213,36,93.6,4},{214,36,100.8,4},{215,36,108.0,4},{216,36,115.2,4},{217,36,122.4,4},{218,36,129.6,4},{219,36,136.8,4},{220,36,144.0,4},{221,36,151.2,4},{222,36,158.4,4},{223,36,165.6,4},{224,36,172.8,4},{225,36,180.0,4},{226,36,187.2,4},{227,36,194.4,4},{228,36,201.6,4},{229,36,208.8,4},{230,36,216.0,4},{231,36,223.2,4},{232,36,230.4,4},{233,36,237.6,4},{234,36,244.8,4},{235,36,252.0,4},{236,36,259.2,4},{237,36,266.4,4},{238,36,273.6,4},{239,36,280.8,4},{240,36,288.0,4},{241,36,295.2,4},{242,36,302.4,4},{243,36,309.6,4},{244,36,316.8,4},{245,36,324.0,4},{246,36,331.2,4},{247,36,338.4,4},{248,36,345.6,4},{249,36,352.8,4}},{{250,45,3.6,5},{251,45,10.8,5},{252,45,18.0,5},{253,45,25.2,5},{254,45,32.4,5},{255,45,39.6,5},{256,45,46.8,5},{257,45,54.0,5},{258,45,61.2,5},{259,45,68.4,5},{260,45,75.6,5},{261,45,82.8,5},{262,45,90.0,5},{263,45,97.2,5},{264,45,104.4,5},{265,45,111.6,5},{266,45,118.8,5},{267,45,126.0,5},{268,45,133.2,5},{269,45,140.4,5},{270,45,147.6,5},{271,45,154.8,5},{272,45,162.0,5},{273,45,169.2,5},{274,45,176.4,5},{275,45,183.6,5},{276,45,190.8,5},{277,45,198.0,5},{278,45,205.2,5},{279,45,212.4,5},{280,45,219.6,5},{281,45,226.8,5},{282,45,234.0,5},{283,45,241.2,5},{284,45,248.4,5},{285,45,255.6,5},{286,45,262.8,5},{287,45,270.0,5},{288,45,277.2,5},{289,45,284.4,5},{290,45,291.6,5},{291,45,298.8,5},{292,45,306.0,5},{293,45,313.2,5},{294,45,320.4,5},{295,45,327.6,5},{296,45,334.8,5},{297,45,342.0,5},{298,45,349.2,5},{299,45,356.4,5}},{{300,54,0.0,6},{301,54,7.2,6},{302,54,14.4,6},{303,54,21.6,6},{304,54,28.8,6},{305,54,36.0,6},{306,54,43.2,6},{307,54,50.4,6},{308,54,57.6,6},{309,54,64.8,6},{310,54,72.0,6},{311,54,79.2,6},{312,54,86.4,6},{313,54,93.6,6},{314,54,100.8,6},{315,54,108.0,6},{316,54,115.2,6},{317,54,122.4,6},{318,54,129.6,6},{319,54,136.8,6},{320,54,144.0,6},{321,54,151.2,6},{322,54,158.4,6},{323,54,165.6,6},{324,54,172.8,6},{325,54,180.0,6},{326,54,187.2,6},{327,54,194.4,6},{328,54,201.6,6},{329,54,208.8,6},{330,54,216.0,6},{331,54,223.2,6},{332,54,230.4,6},{333,54,237.6,6},{334,54,244.8,6},{335,54,252.0,6},{336,54,259.2,6},{337,54,266.4,6},{338,54,273.6,6},{339,54,280.8,6},{340,54,288.0,6},{341,54,295.2,6},{342,54,302.4,6},{343,54,309.6,6},{344,54,316.8,6},{345,54,324.0,6},{346,54,331.2,6},{347,54,338.4,6},{348,54,345.6,6},{349,54,352.8,6}},{{350,63,3.6,7},{351,63,10.8,7},{352,63,18.0,7},{353,63,25.2,7},{354,63,32.4,7},{355,63,39.6,7},{356,63,46.8,7},{357,63,54.0,7},{358,63,61.2,7},{359,63,68.4,7},{360,63,75.6,7},{361,63,82.8,7},{362,63,90.0,7},{363,63,97.2,7},{364,63,104.4,7},{365,63,111.6,7},{366,63,118.8,7},{367,63,126.0,7},{368,63,133.2,7},{369,63,140.4,7},{370,63,147.6,7},{371,63,154.8,7},{372,63,162.0,7},{373,63,169.2,7},{374,63,176.4,7},{375,63,183.6,7},{376,63,190.8,7},{377,63,198.0,7},{378,63,205.2,7},{379,63,212.4,7},{380,63,219.6,7},{381,63,226.8,7},{382,63,234.0,7},{383,63,241.2,7},{384,63,248.4,7},{385,63,255.6,7},{386,63,262.8,7},{387,63,270.0,7},{388,63,277.2,7},{389,63,284.4,7},{390,63,291.6,7},{391,63,298.8,7},{392,63,306.0,7},{393,63,313.2,7},{394,63,320.4,7},{395,63,327.6,7},{396,63,334.8,7},{397,63,342.0,7},{398,63,349.2,7},{399,63,356.4,7}},{{400,72,0.0,8},{401,72,7.2,8},{402,72,14.4,8},{403,72,21.6,8},{404,72,28.8,8},{405,72,36.0,8},{406,72,43.2,8},{407,72,50.4,8},{408,72,57.6,8},{409,72,64.8,8},{410,72,72.0,8},{411,72,79.2,8},{412,72,86.4,8},{413,72,93.6,8},{414,72,100.8,8},{415,72,108.0,8},{416,72,115.2,8},{417,72,122.4,8},{418,72,129.6,8},{419,72,136.8,8},{420,72,144.0,8},{421,72,151.2,8},{422,72,158.4,8},{423,72,165.6,8},{424,72,172.8,8},{425,72,180.0,8},{426,72,187.2,8},{427,72,194.4,8},{428,72,201.6,8},{429,72,208.8,8},{430,72,216.0,8},{431,72,223.2,8},{432,72,230.4,8},{433,72,237.6,8},{434,72,244.8,8},{435,72,252.0,8},{436,72,259.2,8},{437,72,266.4,8},{438,72,273.6,8},{439,72,280.8,8},{440,72,288.0,8},{441,72,295.2,8},{442,72,302.4,8},{443,72,309.6,8},{444,72,316.8,8},{445,72,324.0,8},{446,72,331.2,8},{447,72,338.4,8},{448,72,345.6,8},{449,72,352.8,8}},{{450,81,3.6,9},{451,81,10.8,9},{452,81,18.0,9},{453,81,25.2,9},{454,81,32.4,9},{455,81,39.6,9},{456,81,46.8,9},{457,81,54.0,9},{458,81,61.2,9},{459,81,68.4,9},{460,81,75.6,9},{461,81,82.8,9},{462,81,90.0,9},{463,81,97.2,9},{464,81,104.4,9},{465,81,111.6,9},{466,81,118.8,9},{467,81,126.0,9},{468,81,133.2,9},{469,81,140.4,9},{470,81,147.6,9},{471,81,154.8,9},{472,81,162.0,9},{473,81,169.2,9},{474,81,176.4,9},{475,81,183.6,9},{476,81,190.8,9},{477,81,198.0,9},{478,81,205.2,9},{479,81,212.4,9},{480,81,219.6,9},{481,81,226.8,9},{482,81,234.0,9},{483,81,241.2,9},{484,81,248.4,9},{485,81,255.6,9},{486,81,262.8,9},{487,81,270.0,9},{488,81,277.2,9},{489,81,284.4,9},{490,81,291.6,9},{491,81,298.8,9},{492,81,306.0,9},{493,81,313.2,9},{494,81,320.4,9},{495,81,327.6,9},{496,81,334.8,9},{497,81,342.0,9},{498,81,349.2,9},{499,81,356.4,9}},{{500,90,0.0,10},{501,90,7.2,10},{502,90,14.4,10},{503,90,21.6,10},{504,90,28.8,10},{505,90,36.0,10},{506,90,43.2,10},{507,90,50.4,10},{508,90,57.6,10},{509,90,64.8,10},{510,90,72.0,10},{511,90,79.2,10},{512,90,86.4,10},{513,90,93.6,10},{514,90,100.8,10},{515,90,108.0,10},{516,90,115.2,10},{517,90,122.4,10},{518,90,129.6,10},{519,90,136.8,10},{520,90,144.0,10},{521,90,151.2,10},{522,90,158.4,10},{523,90,165.6,10},{524,90,172.8,10},{525,90,180.0,10},{526,90,187.2,10},{527,90,194.4,10},{528,90,201.6,10},{529,90,208.8,10},{530,90,216.0,10},{531,90,223.2,10},{532,90,230.4,10},{533,90,237.6,10},{534,90,244.8,10},{535,90,252.0,10},{536,90,259.2,10},{537,90,266.4,10},{538,90,273.6,10},{539,90,280.8,10},{540,90,288.0,10},{541,90,295.2,10},{542,90,302.4,10},{543,90,309.6,10},{544,90,316.8,10},{545,90,324.0,10},{546,90,331.2,10},{547,90,338.4,10},{548,90,345.6,10},{549,90,352.8,10}},{{550,99,3.6,11},{551,99,10.8,11},{552,99,18.0,11},{553,99,25.2,11},{554,99,32.4,11},{555,99,39.6,11},{556,99,46.8,11},{557,99,54.0,11},{558,99,61.2,11},{559,99,68.4,11},{560,99,75.6,11},{561,99,82.8,11},{562,99,90.0,11},{563,99,97.2,11},{564,99,104.4,11},{565,99,111.6,11},{566,99,118.8,11},{567,99,126.0,11},{568,99,133.2,11},{569,99,140.4,11},{570,99,147.6,11},{571,99,154.8,11},{572,99,162.0,11},{573,99,169.2,11},{574,99,176.4,11},{575,99,183.6,11},{576,99,190.8,11},{577,99,198.0,11},{578,99,205.2,11},{579,99,212.4,11},{580,99,219.6,11},{581,99,226.8,11},{582,99,234.0,11},{583,99,241.2,11},{584,99,248.4,11},{585,99,255.6,11},{586,99,262.8,11},{587,99,270.0,11},{588,99,277.2,11},{589,99,284.4,11},{590,99,291.6,11},{591,99,298.8,11},{592,99,306.0,11},{593,99,313.2,11},{594,99,320.4,11},{595,99,327.6,11},{596,99,334.8,11},{597,99,342.0,11},{598,99,349.2,11},{599,99,356.4,11}},{{600,108,0.0,12},{601,108,7.2,12},{602,108,14.4,12},{603,108,21.6,12},{604,108,28.8,12},{605,108,36.0,12},{606,108,43.2,12},{607,108,50.4,12},{608,108,57.6,12},{609,108,64.8,12},{610,108,72.0,12},{611,108,79.2,12},{612,108,86.4,12},{613,108,93.6,12},{614,108,100.8,12},{615,108,108.0,12},{616,108,115.2,12},{617,108,122.4,12},{618,108,129.6,12},{619,108,136.8,12},{620,108,144.0,12},{621,108,151.2,12},{622,108,158.4,12},{623,108,165.6,12},{624,108,172.8,12},{625,108,180.0,12},{626,108,187.2,12},{627,108,194.4,12},{628,108,201.6,12},{629,108,208.8,12},{630,108,216.0,12},{631,108,223.2,12},{632,108,230.4,12},{633,108,237.6,12},{634,108,244.8,12},{635,108,252.0,12},{636,108,259.2,12},{637,108,266.4,12},{638,108,273.6,12},{639,108,280.8,12},{640,108,288.0,12},{641,108,295.2,12},{642,108,302.4,12},{643,108,309.6,12},{644,108,316.8,12},{645,108,324.0,12},{646,108,331.2,12},{647,108,338.4,12},{648,108,345.6,12},{649,108,352.8,12}},{{650,117,3.6,13},{651,117,10.8,13},{652,117,18.0,13},{653,117,25.2,13},{654,117,32.4,13},{655,117,39.6,13},{656,117,46.8,13},{657,117,54.0,13},{658,117,61.2,13},{659,117,68.4,13},{660,117,75.6,13},{661,117,82.8,13},{662,117,90.0,13},{663,117,97.2,13},{664,117,104.4,13},{665,117,111.6,13},{666,117,118.8,13},{667,117,126.0,13},{668,117,133.2,13},{669,117,140.4,13},{670,117,147.6,13},{671,117,154.8,13},{672,117,162.0,13},{673,117,169.2,13},{674,117,176.4,13},{675,117,183.6,13},{676,117,190.8,13},{677,117,198.0,13},{678,117,205.2,13},{679,117,212.4,13},{680,117,219.6,13},{681,117,226.8,13},{682,117,234.0,13},{683,117,241.2,13},{684,117,248.4,13},{685,117,255.6,13},{686,117,262.8,13},{687,117,270.0,13},{688,117,277.2,13},{689,117,284.4,13},{690,117,291.6,13},{691,117,298.8,13},{692,117,306.0,13},{693,117,313.2,13},{694,117,320.4,13},{695,117,327.6,13},{696,117,334.8,13},{697,117,342.0,13},{698,117,349.2,13},{699,117,356.4,13}},{{700,126,0.0,14},{701,126,7.2,14},{702,126,14.4,14},{703,126,21.6,14},{704,126,28.8,14},{705,126,36.0,14},{706,126,43.2,14},{707,126,50.4,14},{708,126,57.6,14},{709,126,64.8,14},{710,126,72.0,14},{711,126,79.2,14},{712,126,86.4,14},{713,126,93.6,14},{714,126,100.8,14},{715,126,108.0,14},{716,126,115.2,14},{717,126,122.4,14},{718,126,129.6,14},{719,126,136.8,14},{720,126,144.0,14},{721,126,151.2,14},{722,126,158.4,14},{723,126,165.6,14},{724,126,172.8,14},{725,126,180.0,14},{726,126,187.2,14},{727,126,194.4,14},{728,126,201.6,14},{729,126,208.8,14},{730,126,216.0,14},{731,126,223.2,14},{732,126,230.4,14},{733,126,237.6,14},{734,126,244.8,14},{735,126,252.0,14},{736,126,259.2,14},{737,126,266.4,14},{738,126,273.6,14},{739,126,280.8,14},{740,126,288.0,14},{741,126,295.2,14},{742,126,302.4,14},{743,126,309.6,14},{744,126,316.8,14},{745,126,324.0,14},{746,126,331.2,14},{747,126,338.4,14},{748,126,345.6,14},{749,126,352.8,14}},{{750,135,3.6,15},{751,135,10.8,15},{752,135,18.0,15},{753,135,25.2,15},{754,135,32.4,15},{755,135,39.6,15},{756,135,46.8,15},{757,135,54.0,15},{758,135,61.2,15},{759,135,68.4,15},{760,135,75.6,15},{761,135,82.8,15},{762,135,90.0,15},{763,135,97.2,15},{764,135,104.4,15},{765,135,111.6,15},{766,135,118.8,15},{767,135,126.0,15},{768,135,133.2,15},{769,135,140.4,15},{770,135,147.6,15},{771,135,154.8,15},{772,135,162.0,15},{773,135,169.2,15},{774,135,176.4,15},{775,135,183.6,15},{776,135,190.8,15},{777,135,198.0,15},{778,135,205.2,15},{779,135,212.4,15},{780,135,219.6,15},{781,135,226.8,15},{782,135,234.0,15},{783,135,241.2,15},{784,135,248.4,15},{785,135,255.6,15},{786,135,262.8,15},{787,135,270.0,15},{788,135,277.2,15},{789,135,284.4,15},{790,135,291.6,15},{791,135,298.8,15},{792,135,306.0,15},{793,135,313.2,15},{794,135,320.4,15},{795,135,327.6,15},{796,135,334.8,15},{797,135,342.0,15},{798,135,349.2,15},{799,135,356.4,15}},{{800,144,0.0,16},{801,144,7.2,16},{802,144,14.4,16},{803,144,21.6,16},{804,144,28.8,16},{805,144,36.0,16},{806,144,43.2,16},{807,144,50.4,16},{808,144,57.6,16},{809,144,64.8,16},{810,144,72.0,16},{811,144,79.2,16},{812,144,86.4,16},{813,144,93.6,16},{814,144,100.8,16},{815,144,108.0,16},{816,144,115.2,16},{817,144,122.4,16},{818,144,129.6,16},{819,144,136.8,16},{820,144,144.0,16},{821,144,151.2,16},{822,144,158.4,16},{823,144,165.6,16},{824,144,172.8,16},{825,144,180.0,16},{826,144,187.2,16},{827,144,194.4,16},{828,144,201.6,16},{829,144,208.8,16},{830,144,216.0,16},{831,144,223.2,16},{832,144,230.4,16},{833,144,237.6,16},{834,144,244.8,16},{835,144,252.0,16},{836,144,259.2,16},{837,144,266.4,16},{838,144,273.6,16},{839,144,280.8,16},{840,144,288.0,16},{841,144,295.2,16},{842,144,302.4,16},{843,144,309.6,16},{844,144,316.8,16},{845,144,324.0,16},{846,144,331.2,16},{847,144,338.4,16},{848,144,345.6,16},{849,144,352.8,16}},{{850,153,3.6,17},{851,153,10.8,17},{852,153,18.0,17},{853,153,25.2,17},{854,153,32.4,17},{855,153,39.6,17},{856,153,46.8,17},{857,153,54.0,17},{858,153,61.2,17},{859,153,68.4,17},{860,153,75.6,17},{861,153,82.8,17},{862,153,90.0,17},{863,153,97.2,17},{864,153,104.4,17},{865,153,111.6,17},{866,153,118.8,17},{867,153,126.0,17},{868,153,133.2,17},{869,153,140.4,17},{870,153,147.6,17},{871,153,154.8,17},{872,153,162.0,17},{873,153,169.2,17},{874,153,176.4,17},{875,153,183.6,17},{876,153,190.8,17},{877,153,198.0,17},{878,153,205.2,17},{879,153,212.4,17},{880,153,219.6,17},{881,153,226.8,17},{882,153,234.0,17},{883,153,241.2,17},{884,153,248.4,17},{885,153,255.6,17},{886,153,262.8,17},{887,153,270.0,17},{888,153,277.2,17},{889,153,284.4,17},{890,153,291.6,17},{891,153,298.8,17},{892,153,306.0,17},{893,153,313.2,17},{894,153,320.4,17},{895,153,327.6,17},{896,153,334.8,17},{897,153,342.0,17},{898,153,349.2,17},{899,153,356.4,17}},{{900,162,0.0,18},{901,162,7.2,18},{902,162,14.4,18},{903,162,21.6,18},{904,162,28.8,18},{905,162,36.0,18},{906,162,43.2,18},{907,162,50.4,18},{908,162,57.6,18},{909,162,64.8,18},{910,162,72.0,18},{911,162,79.2,18},{912,162,86.4,18},{913,162,93.6,18},{914,162,100.8,18},{915,162,108.0,18},{916,162,115.2,18},{917,162,122.4,18},{918,162,129.6,18},{919,162,136.8,18},{920,162,144.0,18},{921,162,151.2,18},{922,162,158.4,18},{923,162,165.6,18},{924,162,172.8,18},{925,162,180.0,18},{926,162,187.2,18},{927,162,194.4,18},{928,162,201.6,18},{929,162,208.8,18},{930,162,216.0,18},{931,162,223.2,18},{932,162,230.4,18},{933,162,237.6,18},{934,162,244.8,18},{935,162,252.0,18},{936,162,259.2,18},{937,162,266.4,18},{938,162,273.6,18},{939,162,280.8,18},{940,162,288.0,18},{941,162,295.2,18},{942,162,302.4,18},{943,162,309.6,18},{944,162,316.8,18},{945,162,324.0,18},{946,162,331.2,18},{947,162,338.4,18},{948,162,345.6,18},{949,162,352.8,18}},{{950,171,3.6,19},{951,171,10.8,19},{952,171,18.0,19},{953,171,25.2,19},{954,171,32.4,19},{955,171,39.6,19},{956,171,46.8,19},{957,171,54.0,19},{958,171,61.2,19},{959,171,68.4,19},{960,171,75.6,19},{961,171,82.8,19},{962,171,90.0,19},{963,171,97.2,19},{964,171,104.4,19},{965,171,111.6,19},{966,171,118.8,19},{967,171,126.0,19},{968,171,133.2,19},{969,171,140.4,19},{970,171,147.6,19},{971,171,154.8,19},{972,171,162.0,19},{973,171,169.2,19},{974,171,176.4,19},{975,171,183.6,19},{976,171,190.8,19},{977,171,198.0,19},{978,171,205.2,19},{979,171,212.4,19},{980,171,219.6,19},{981,171,226.8,19},{982,171,234.0,19},{983,171,241.2,19},{984,171,248.4,19},{985,171,255.6,19},{986,171,262.8,19},{987,171,270.0,19},{988,171,277.2,19},{989,171,284.4,19},{990,171,291.6,19},{991,171,298.8,19},{992,171,306.0,19},{993,171,313.2,19},{994,171,320.4,19},{995,171,327.6,19},{996,171,334.8,19},{997,171,342.0,19},{998,171,349.2,19},{999,171,356.4,19}}};


  // 设置空卫星节点
  NodeContainer nodes;
  // nodes.Create (66);
  nodes.Create(1000);

  // 添加卫星进去
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::LEOSatelliteMobilityModel");
  mobility.InstallAll ();
  int index = 0;
  // iterate our nodes and print their position.
  for (NodeContainer::Iterator j = nodes.Begin ();j != nodes.End (); ++j){
      Ptr<Node> object = *j;
      Ptr<LEOSatelliteMobilityModel> position = object->GetObject<LEOSatelliteMobilityModel> ();
    //   position->setFileName("iridium2-2021-3-9-2.txt");// @suppress("Invalid arguments") // 记录卫星运动的文件名
      // 设置每个卫星的初始位置
      struct LEOSatPolarPos pPos;
      pPos.planeNum = planeNum;
      pPos.satelliteNumInOnePlane = satelliteInOnePlane;
      pPos.altitude = ALTITUDE;
      pPos.longitude =  iridiumConstellation[index/satelliteInOnePlane][index%satelliteInOnePlane][1];
      pPos.alpha =  iridiumConstellation[index/satelliteInOnePlane][index%satelliteInOnePlane][2];
      pPos.inclination =  INCLINATION;
      pPos.plane =  iridiumConstellation[index/satelliteInOnePlane][index%satelliteInOnePlane][3];
      pPos.self = nodes.Get(index);
      pPos.index = index;
      // pPos.planeNum = 6;
      // pPos.satelliteNumInOnePlane = 11;// @suppress("Field cannot be resolved")
      // 设置每个卫星的相邻卫星节点
      // 同轨道的上下节点
      if(index%satelliteInOnePlane == 0){
    	  pPos.before = nodes.Get(index+1);
    	  pPos.after = nodes.Get(index + satelliteInOnePlane - 1);
      }else if(index%satelliteInOnePlane== satelliteInOnePlane - 1){
    	  pPos.before = nodes.Get(index-satelliteInOnePlane + 1);
    	  pPos.after = nodes.Get(index-1);
      }else{
    	  pPos.before = nodes.Get(index+1);
    	  pPos.after = nodes.Get(index-1);
      }
      // 不同轨道的左右节点
      if(index/satelliteInOnePlane == 0){
    	  // 最左边的轨道，这里的卫星的左边卫星是在seam的另一侧，所以它们的left为空
    	  pPos.left = NULL;
    	  pPos.right = nodes.Get(index+satelliteInOnePlane);
      }else if(index/satelliteInOnePlane == planeNum-1){
    	  // 最右边的轨道，这里的卫星的右边卫星是在seam的另一侧，所以它们的right为空
    	  pPos.left = nodes.Get(index-satelliteInOnePlane);
    	  pPos.right = NULL;
      }else{
    	  pPos.left = nodes.Get(index-satelliteInOnePlane);
    	  pPos.right = nodes.Get(index+satelliteInOnePlane);
      }
      position->SetSatSphericalPos(pPos); // @suppress("Invalid arguments")
      position->setRoutingAlgorithmAndSnapShotWay(2, 2); // @suppress("Invalid arguments")
      NS_ASSERT (position != 0);
      index++;
    }

  // NixHelper to install nix-vector routing on all nodes
  Ipv4NixVectorHelper nixRouting;
  Ipv4StaticRoutingHelper staticRouting;
  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  list.Add (nixRouting, 10);
  InternetStackHelper stack;
  stack.SetRoutingHelper (list); // has effect on the next Install ()
  stack.Install (nodes);

  // 建立链路，分配ip地址
  // 先分配同一轨道的ip地址:从10.1.1.0开始分配，子网掩码是255.255.0.0
  index = 0;
  int ipIndex = 1;
  PointToPointHelper p2p;
  Ipv4AddressHelper ipv4;
  // 数组，用来保存链路分配的ip地址，顺序存放
  // Ipv4InterfaceContainer ipv4InterfaceContainer[121];
  Ipv4InterfaceContainer ipv4InterfaceContainer[2048];
  while(index<satelliteInOnePlane*planeNum){
	  // 每个卫星只负责和它上面的卫星建立链接并分配ip
	  int nextNodeIndex;// 该卫星上面的卫星的索引
	  if(index%satelliteInOnePlane != satelliteInOnePlane-1){
		  nextNodeIndex = index+1;
	  } else {
		  nextNodeIndex = index-satelliteInOnePlane + 1;
	  }
	  NodeContainer nodeContainer = NodeContainer (nodes.Get (index), nodes.Get (nextNodeIndex));
	  // 卫星延时时间计算：
	  // 同一个轨道前后卫星距离 d =（海拔+地球半径）*sin（32.72/2）*2*1000 = 4032411.4931 m
	  // 卫星间激光通信： v = 299792458 m/s
	  // delay = d / v = 13 ms
	  p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
	  p2p.SetChannelAttribute ("Delay", StringValue ("13ms"));
	  NetDeviceContainer netDeviceContainer = p2p.Install (nodeContainer);

	  // std::string ip = "10.1." + std::to_string(ipIndex) + ".0";
    std::string ip = "10." + std::to_string(ipIndex/256+1) + "." + std::to_string((ipIndex - 1) % 256) + ".0";
	  const char* ipp = ip.c_str();
	  // std::cout<< ip << std::endl;
	  ipv4.SetBase (ipp, "255.255.255.0");
	  Ipv4InterfaceContainer ipv4Container = ipv4.Assign (netDeviceContainer);
	  ipv4InterfaceContainer[ipIndex-1] = ipv4Container;
	  index++;
	  ipIndex++;
  }
  //再分配不同轨道的ip地址
  index = 0;
  while(index<(satelliteInOnePlane*(planeNum - 1))){
	  //每个卫星只负责和它左边的卫星建立链接并分配ip
	  NodeContainer nodeContainer = NodeContainer (nodes.Get (index), nodes.Get (index+satelliteInOnePlane));

	  p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
	  // 卫星延时时间计算：不同轨道间的卫星之间距离一直在变，这里仿真先直接设为定值13ms，后期再改
	  p2p.SetChannelAttribute ("Delay", StringValue ("13ms"));
	  NetDeviceContainer netDeviceContainer = p2p.Install (nodeContainer);

	  // std::string ip = "10.1." + std::to_string(ipIndex) + ".0";
    std::string ip = "10." + std::to_string(ipIndex/256 + 1) + "." + std::to_string((ipIndex - 1) % 256) + ".0";
	  const char* ipp = ip.c_str();
	  // std::cout<< ip << std::endl;
	  ipv4.SetBase (ipp, "255.255.255.0");
	  Ipv4InterfaceContainer ipv4Container = ipv4.Assign (netDeviceContainer);
	  ipv4InterfaceContainer[ipIndex-1] = ipv4Container;
	  index++;
	  ipIndex++;
  }

  // 卫星周期计算公式： T = sqrt((4 * (PI)^2 * (r + h)^3 )/ (G * M ))  , 其中 G * M = 398601.2 (km^3/s^2)
  // irrdium : T = sqrt((4 * (3.1415926535897)^2 * (6378 + 780)^3 )/ 398601.2) = 6026.957216098

  //  Ipv4InterfaceContainer i0i1 = ipv4InterfaceContainer[0];//卫星编号0和1之间的链路的ip分配
  Ipv4InterfaceContainer i1i2 = ipv4InterfaceContainer[1];//卫星编号1和2之间的链路的ip分配
  Ipv4InterfaceContainer i11i12 = ipv4InterfaceContainer[11];//卫星编号11和12之间的链路的ip分配
  Ipv4InterfaceContainer i12i13 = ipv4InterfaceContainer[12];//卫星编号12和13之间的链路的ip分配
  Ipv4InterfaceContainer i23i34 = ipv4InterfaceContainer[90];//卫星编号23和34之间的链路的ip分配
  Ipv4InterfaceContainer i0i11 = ipv4InterfaceContainer[66];//卫星编号0和11之间的链路的ip分配
  Ipv4InterfaceContainer i1i12 = ipv4InterfaceContainer[67];//卫星编号1和12之间的链路的ip分配
  Ipv4InterfaceContainer i2i13 = ipv4InterfaceContainer[68];//卫星编号2和13之间的链路的ip分配
  Ipv4InterfaceContainer i33i34 = ipv4InterfaceContainer[33];//卫星编号33和34之间的链路的ip分配

//  // 实验1：s=22，d=1，t=186.22
//  Ptr<Node> source = nodes.Get(22);
//  Ptr<Node> dest = nodes.Get(1);
//  Ipv4Address destinationIp = i1i2.GetAddress (0);
//  double serverStartTime = 170.0;
//  double serverEndTime = 215.0;
//  double clientStartTime = 180.0;
//  double clientEndTime = 215.0;
//  double interval = 0.1;

//  UdpEchoServerHelper echoServer (9);
//  ApplicationContainer serverApps = echoServer.Install (dest);
//  serverApps.Start (Seconds (serverStartTime));
//  serverApps.Stop (Seconds (serverEndTime));

//  UdpEchoClientHelper echoClient (destinationIp, 9);
//  echoClient.SetAttribute ("MaxPackets", UintegerValue (200));
//  echoClient.SetAttribute ("Interval", TimeValue (Seconds (interval)));
//  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

//  ApplicationContainer clientApps = echoClient.Install (source);
//  clientApps.Start (Seconds (clientStartTime));
//  clientApps.Stop (Seconds (clientEndTime));

//  Simulator::Schedule (Seconds (clientStartTime-0.05), Progress,destinationIp,source,dest,interval,clientEndTime);

// 	// 航点发生故障
//  Simulator::Schedule (Seconds (184.72), setFaultSatellite, nodes.Get(12), true);
//  Simulator::Schedule (Seconds (186.00), setFaultSatellite, nodes.Get(12), false);
//  //上面设置的注意点：这里设置时间时，注意如果故障卫星恢复的时间在将断链路断开的时间之后，则会转变成非航点卫星故障，而非航点卫星故障并不一定能找到新的航点，即不一定有新路径，如果找不到，程序会报错（后期解决）



//  // 实验2：s=24，d=1，t=186.22
//  Ptr<Node> source2 = nodes.Get(24);
//  Ptr<Node> dest2 = nodes.Get(1);
//  Ipv4Address destinationIp2 = i1i2.GetAddress (0);
//  double serverStartTime2 = 170.0;
//  double serverEndTime2 = 215.0;
//  double clientStartTime2 = 181.0;
//  double clientEndTime2 = 215.0;
//  double interval2 = 0.1;

//  UdpEchoServerHelper echoServer2 (9);
//  ApplicationContainer serverApps2 = echoServer2.Install (dest2);
//  serverApps2.Start (Seconds (serverStartTime2));
//  serverApps2.Stop (Seconds (serverEndTime2));

//  UdpEchoClientHelper echoClient2 (destinationIp2, 9);
//  echoClient2.SetAttribute ("MaxPackets", UintegerValue (200));
//  echoClient2.SetAttribute ("Interval", TimeValue (Seconds (interval2)));
//  echoClient2.SetAttribute ("PacketSize", UintegerValue (1024));

//  ApplicationContainer clientApps2 = echoClient2.Install (source2);
//  clientApps2.Start (Seconds (clientStartTime2));
//  clientApps2.Stop (Seconds (clientEndTime2));

//  Simulator::Schedule (Seconds (clientStartTime2-0.05), Progress,destinationIp2,source2,dest2,interval2,clientEndTime2);

// 	// 航点发生故障
//  Simulator::Schedule (Seconds (184.72), setFaultSatellite, nodes.Get(0), true);
//  Simulator::Schedule (Seconds (186.00), setFaultSatellite, nodes.Get(0), false);
//  //上面设置的注意点：这里设置时间时，注意如果故障卫星恢复的时间在将断链路断开的时间之后，则会转变成非航点卫星故障，而非航点卫星故障并不一定能找到新的航点，即不一定有新路径，如果找不到，程序会报错（后期解决）


//  // 实验3：s=5，d=33，t=734.17
  Ptr<Node> source3 = nodes.Get(5);//！！！！
  Ptr<Node> dest3 = nodes.Get(33);
  Ipv4Address destinationIp3 = i33i34.GetAddress (0);
  double serverStartTime3 = 720.0;
  double serverEndTime3 = 800.0;
  double clientStartTime3 = 728.0;
  double clientEndTime3 = 800.0;
  double interval3 = 0.1;  //

  UdpEchoServerHelper echoServer3 (9);
  ApplicationContainer serverApps3 = echoServer3.Install (dest3);
  serverApps3.Start (Seconds (serverStartTime3));
  serverApps3.Stop (Seconds (serverEndTime3));

  UdpEchoClientHelper echoClient3 (destinationIp3, 9);
  echoClient3.SetAttribute ("MaxPackets", UintegerValue (200)); //
  echoClient3.SetAttribute ("Interval", TimeValue (Seconds (interval3)));
  echoClient3.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps3 = echoClient3.Install (source3);
  clientApps3.Start (Seconds (clientStartTime3));
  clientApps3.Stop (Seconds (clientEndTime3));

  Simulator::Schedule (Seconds (clientStartTime3-0.05), Progress,destinationIp3,source3,dest3,interval3,clientEndTime3);

// // 非航点发生故障
//  Simulator::Schedule (Seconds (729.85), setFaultSatellite, nodes.Get(11), true); //728.85
//  Simulator::Schedule (Seconds (732.85), setFaultSatellite, nodes.Get(11), false); //729.85
//  Simulator::Schedule (Seconds (729.85), setFaultSatellite, nodes.Get(21), true); //730.85
//  Simulator::Schedule (Seconds (732.85), setFaultSatellite, nodes.Get(21), false); //732.85
// // 航点发生故障
//	Simulator::Schedule (Seconds (731.85), setFaultSatellite, nodes.Get(10), true);
//	Simulator::Schedule (Seconds (733.85), setFaultSatellite, nodes.Get(10), false);

  double simulatorLastTime = 800.0;
  newSnapshotWay(nodes, simulatorLastTime, 6026.957216098);
  Simulator::Stop (Seconds (simulatorLastTime));
  AsciiTraceHelper ascii;
  p2p.EnableAsciiAll (ascii.CreateFileStream ("test02-SWS-false.tr"));
//  p2p.EnablePcapAll ("iridium-SWS");

  Simulator::Run ();
  Simulator::Destroy ();

  // 当前结束时间
  t = time(0);
  strftime( tmp, sizeof(tmp), "%Y/%m/%d %X" ,localtime(&t) );
  puts( tmp );
}
