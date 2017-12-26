//***************//
#include <iostream>
#include <include/MavConnPx4Flow.h>
using namespace std;
MAVCONNPx4Flow::MAVCONNPx4Flow():system_id(1),
    companion_id(1),
    quit(false)
{

}
MAVCONNPx4Flow::MAVCONNPx4Flow(const char* name, int baudrate ):sp(name, baudrate),
    system_id(1),
    companion_id(1),
    quit(false)
{

}

MAVCONNPx4Flow::~MAVCONNPx4Flow()
{

}

void MAVCONNPx4Flow::open(){
    sp.start();
}

void MAVCONNPx4Flow::_readingLoop(){
    int i=0;
    while(!quit){
        mavlink_message_t msg;
        int nMsg = sp.read_message(msg);
        if( nMsg > 0){
            if( msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW){
                std::cout << "sysid:" << (int) msg.sysid << " " << (int) msg.compid << " msgid:" << (int) msg.msgid << std::endl;
                mavlink_optical_flow_t optical_flow;
                mavlink_msg_optical_flow_decode(&msg,&optical_flow);
                std::cout<<"flow_x: "<<optical_flow.flow_x<<" flow_y: "<<optical_flow.flow_y<<std::endl;
                std::cout<<"ground_distance: "<<optical_flow.ground_distance<<std::endl;
                ++i;
                std::cout<<i<<"-th read succesful!"<<std::endl;
            }

        }
        //sleep(1);
    }
}

void* startMAVCONNPx4FlowReadThread(void *args)
{
    // takes an autopilot object argument
    MAVCONNPx4Flow *mav_conn = (MAVCONNPx4Flow*)args;
    // run the object's read thread
    mav_conn->_readingLoop();
    // done!

    return NULL;
}
void MAVCONNPx4Flow::startReadingLoop(){
   pthread_create( &read_tid, NULL, &startMAVCONNPx4FlowReadThread, this );
   if(MAVCONNPx4Flow::quit)
       cout<<"quit=true"<<endl;
   else
       cout<<"quit=false"<<endl;
}


