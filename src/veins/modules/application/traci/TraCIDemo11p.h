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

#ifndef TraCIDemo11p_H
#define TraCIDemo11p_H
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

/**
 * @brief
 * A tutorial demo for TraCI. When the car is stopped for longer than 10 seconds
 * it will send a message out to other cars containing the blocked road id.
 * Receiving cars will then trigger a reroute via TraCI.
 * When channel switching between SCH and CCH is enabled on the MAC, the message is
 * instead send out on a service channel following a WAVE Service Advertisement
 * on the CCH.
 *
 * @author Christoph Sommer : initial DemoApp
 * @author David Eckhoff : rewriting, moving functionality to BaseWaveApplLayer, adding WSA
 *
 */


// MAHBOUBEH

class TraCIDemo11p : public BaseWaveApplLayer {
	private:
    cMessage *timeoutEvent1;
    simtime_t timeout1;
    int a;
	public:
		virtual void initialize(int stage);

	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;
		Mac1609_4* mac1;
<<<<<<< HEAD

	protected:
        virtual void onWSM(WaveShortMessage* wsm);
        virtual void onWSA(WaveServiceAdvertisment* wsa);

        virtual void handleSelfMsg(cMessage* msg);
		virtual void handlePositionUpdate(cObject* obj);

		virtual void onBSM(BasicSafetyMessage* bsm);
		virtual void finish();
=======
		
	protected:
        virtual void onWSM(WaveShortMessage* wsm);
        virtual void onWSA(WaveServiceAdvertisment* wsa);
       
        virtual void handleSelfMsg(cMessage* msg);
	virtual void handlePositionUpdate(cObject* obj);
		
	virtual void onBSM(BasicSafetyMessage* bsm);
	virtual void finish();
>>>>>>> 45ddc2c28ff926d93c5a4bdcb2279369f7edcb03
};

#endif
