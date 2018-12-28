//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/modules/application/traci/TraCIDemo11p.h"
#include <fstream>
#include <iostream>
using namespace std;
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;
using Veins::TraCIScenarioManager;
using Veins::TraCICommandInterface;


struct NeighborInfo
{
    string RoadId;
    double speed;
    long pdr;

};
struct meanNInfo
{
    double speed;
    int density;
    long pdr;
};

struct unn
{
    string s;
};
Define_Module(TraCIDemo11p);

map<pair<string,string>,NeighborInfo> NeighborTables; //key is nId and time
map<pair<string,string>,meanNInfo> MeanNeighborInfo; //key is RoadId and time
map<pair<string,string>,unn> uni1; //key is vehId and time

ofstream test1;

void TraCIDemo11p::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;

        //timeoutEvent = nullptr;
        timeout1 = 20.0;
        //timeoutEvent1 = new cMessage("timeoutEvent1");
        //scheduleAt(simTime()+timeout1, timeoutEvent1);
    }

}

void TraCIDemo11p::onWSA(WaveServiceAdvertisment* wsa) {
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(wsa->getTargetChannel());
        currentSubscribedServiceId = wsa->getPsid();
        if  (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService((Channels::ChannelNumber) wsa->getTargetChannel(), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}

void TraCIDemo11p::onWSM(WaveShortMessage* wsm) {
    findHost()->getDisplayString().updateWith("r=16,green");

    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
    if (!sentMessage) {
        sentMessage = true;
        //repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + uniform(0.01,0.2), wsm->dup());
    }
}

void TraCIDemo11p::handleSelfMsg(cMessage* msg) {

    if (msg == timeoutEvent1) {

       //string vId = bsm->getExternalId();
       meanNInfo ds;
       string time=simTime().str();
       string RoadId=mobility->getRoadId();
       std::list<std::string> listRoadIds= traci->getRouteIds();
       std::list<std::string> roads;
       std::list<std::string> uniRoads;
       std::list<std::string> nei;
       std::list<std::string> uniNei;
       double means= traci->road(RoadId).getMeanSpeed();
       //vector<string> roads={};
       //vector<string>::iterator itvector={};
       //int a =
       for(map<pair<string,string>,NeighborInfo>::iterator it= NeighborTables.begin(); it != NeighborTables.end() ; it++)
       {
              roads.push_back(it->second.RoadId);
              roads.push_back(it->second.RoadId);
       }
       roads.unique();
       double meanSpeed=0;
       for(list<string>::iterator listit=uniRoads.begin(); listit!=uniRoads.end(); listit++)
       {
           for(map<pair<string,string>,NeighborInfo>::iterator it= NeighborTables.begin(); it != NeighborTables.end() ; it++)
             {
               if (listit->data() == it->second.RoadId)
                  {
                      nei.push_back(it->first.first);
                      meanSpeed += it->second.speed;
                  }


              }
           nei.unique();
           int n= uniNei.size();
           ds.density = n;
           ds.speed= meanSpeed/n;
           ds.pdr= receivedBSMs1/ generatedBSMs1 ;
           MeanNeighborInfo[make_pair(listit->data(), time)]=ds;
       }
       //std::list<std::string> listRoadIds= traci->getRouteIds();


       //neighbor.clear();

       NeighborTables.clear();
       receivedBSMs1= generatedBSMs1=0;
       mac1->statsReceivedPackets1=0;
       mac1->statsReceivedBroadcasts1=0;
       mac1->statsSentPackets1=0;

       cancelEvent(timeoutEvent1);
       //timeoutEvent1 = new cMessage("timeoutEvent1");
       scheduleAt(simTime()+timeout1, timeoutEvent1);

   }

    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) {
        //send this message on the service channel until the counter is 3 or higher.
        //this code only runs when channel switching is enabled

        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() +1);
        if (wsm->getSerial() >= 3) {
            //stop service advertisements
            stopService();
            delete(wsm);
        }
        else {
            scheduleAt(simTime()+1, wsm);
        }
    }
    else {
        // just send BSM Or WSA
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
}

void TraCIDemo11p::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);

    // stopped for at least 10s?
    if (mobility->getSpeed() < 1) {
        if (simTime() - lastDroveAt >= 10 && sentMessage == false) {
            findHost()->getDisplayString().updateWith("r=16,red");
            sentMessage = true;

            WaveShortMessage* wsm = new WaveShortMessage();
            populateWSM(wsm);
            wsm->setWsmData(mobility->getRoadId().c_str());

            //host is standing still due to crash
            if (dataOnSch) {
                startService(Channels::SCH2, 42, "Traffic Information Service");
                //started service and server advertising, schedule message to self to send later
                scheduleAt(computeAsynchronousSendingTime(1,type_SCH),wsm);
            }
            else {
                //send right away on CCH, because channel switching is disabled
                sendDown(wsm);
            }
        }
    }
    else {
        lastDroveAt = simTime();
    }

}

void TraCIDemo11p::onBSM(BasicSafetyMessage* bsm)
{
    //m string VehicleId=mobility->getExternalId();
    //m string vId = bsm->getExternalId();
    //m NeighborInfo neighbor;
    //m string time=simTime().str();
    //m neighbor.speed= bsm->getSenderSpeed1();
    //m neighbor.RoadId=mobility->getRoadId();
    //m NeighborTables[make_pair(vId, time)]=neighbor;



    //myId : is calling module Id
    //traci->road(mobility->getRoadId().c_str()).getMeanSpeed();
    //msg->getId();
    //Coord dir=mobility->getCurrentDirection();


/*
    string vId = bsm->getExternalId();
    unn temp;
    string time=simTime().str();
    temp.s= time;
    string tStamp= bsm->getTimestamp().str();
    uni1[make_pair(vId,tStamp)] = temp;
*/

}

void TraCIDemo11p::finish()
{
    //test1.open("f:/ds.txt",ios::app);

    //for(map<pair<string,string>,meanNInfo>::iterator it= MeanNeighborInfo.begin(); it != MeanNeighborInfo.end() ; it++)
    //{
       //test1<<mobility->getExternalId()<<"     "<<it->first.first<<"     "<<it->first.second <<"    "<<it->second.density<<"   "<<it->second.speed<<"   "<<it->second.pdr <<endl;
    //}
    //test1.close();

    string VehicleId=mobility->getExternalId();
    string nameR= "f:/p_dbcc/c/file_" + VehicleId+ ".txt";
    std::ifstream ifs(nameR);
    string nameW= "f:/p_dbcc/g/file_" + VehicleId+ ".csv";
    test1.open(nameW,ios::app);
    double row;
    int i=1;
    int j=0;
    std::string line;
    //for(int i=1; i!=5; ++i)
    //{
        while(std::getline(ifs, line)) // read one line from ifs
            {
            std::istringstream iss(line); // access line as a stream

            if(j==0)
            {

                iss >> row; // no need to read further
                //double irt= row - rowi;
                j++;
            }else
            {
                iss >> row; // no need to read further
                test1<<row <<"     ,       ";
                ++i;
                    if(i==5)
                    {
                        test1<<endl;
                        i=1;
                    }
            }
            }

        //rowi=row;
    //}
       ifs.close();
       test1.close();

/*
    string VehicleId=mobility->getExternalId();
    string name= "f:/p_dbcc/d/file_" + VehicleId+ ".txt";
        test1.open(name,ios::app);
        for(map<pair<string,string>,unn>::iterator it= uni1.begin(); it != uni1.end() ; it++)
           {
            test1<< it->first.first<<"   "<<it->first.second<<"    "<< it->second.s<<endl;
           }
        test1.close();
        */
}
