#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include <cnoid/Body>
#include <cnoid/Link>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
using namespace cnoid;

class RealModelBallController : public SimpleController {
    Body* body = nullptr;
    Link *left = nullptr, *right = nullptr;
    double dt = 0, t = 0, previousLeft = 0, previousRight = 0;
    bool gripOnly = false, reported = false;
    std::vector<double> start, lift;
    std::ostream* log = nullptr;
    static double blend(double a, double b, double t){
        double u = std::max(0.0, std::min(1.0, (t-a)/(b-a)));
        return u*u*(3.0-2.0*u);
    }
    static double limit(double x, double cap){ return std::max(-cap, std::min(cap, x)); }
public:
    bool initialize(SimpleControllerIO* io) override {
        body=io->body(); log=&io->os(); dt=io->timeStep();
        left=body->link("LHAND_L"); right=body->link("LHAND_R");
        Link* chest=body->link("CHEST_JOINT0");
        Link* wrist=body->link("LARM_JOINT5");
        if(!left || !right || !chest || !wrist || dt<=0){
            *log << "Load the supplied main_dualarm_regrasp.wrl model.\n"; return false;
        }
        std::string option=io->optionString();
        if(option.empty() || option=="pickup") gripOnly=false;
        else if(option=="grip-only") gripOnly=true;
        else { *log << "Option must be grip-only or pickup.\n"; return false; }
        start.resize(body->numJoints());
        for(int i=0;i<body->numJoints();++i) start[i]=body->joint(i)->q();
        body->calcForwardKinematics();
        if(gripOnly) lift=start;
        else {
            Position pose=wrist->T();
            pose.translation().z()+=0.08;
            JointPath arm(chest,wrist);
            if(!arm.calcInverseKinematics(pose)){
                *log << "The 80 mm lift is unreachable; run setup_scene.py again.\n";
                return false;
            }
            lift.resize(start.size());
            for(int i=0;i<body->numJoints();++i) lift[i]=body->joint(i)->q();
        }
        for(int i=0;i<body->numJoints();++i) body->joint(i)->q()=start[i];
        body->calcForwardKinematics();
        for(int i=0;i<body->numJoints();++i){
            Link* j=body->joint(i);
            if(j==left || j==right){ j->setActuationMode(Link::JOINT_FORCE); io->enableIO(j); }
            else { j->setActuationMode(Link::JOINT_DISPLACEMENT); io->enableOutput(j); }
        }
        previousLeft=left->q(); previousRight=right->q(); t=0; reported=false;
        *log << "Real-model practice-ball " << (gripOnly?"grip-only":"pickup")
             << " started. Finger opening +25/-25 mm; closing toward zero.\n";
        return true;
    }
    bool control() override {
        t+=dt;
        double up=gripOnly?0:blend(2.4,4.4,t);
        for(int i=0;i<body->numJoints();++i){
            if(body->joint(i)!=left && body->joint(i)!=right)
                body->joint(i)->q_target()=start[i]+up*(lift[i]-start[i]);
        }
        double open=1.0-blend(0.25,2.0,t);
        const double stiffness=200.0, damping=5.0, forceCap=2.0;
        double vl=(left->q()-previousLeft)/dt, vr=(right->q()-previousRight)/dt;
        left->u()=limit(stiffness*(0.025*open-left->q())-damping*vl,forceCap);
        right->u()=limit(stiffness*(-0.025*open-right->q())-damping*vr,forceCap);
        previousLeft=left->q(); previousRight=right->q();
        if(!reported && t>=2.2){
            *log << "After close: LHAND_L=" << left->q() << " m, LHAND_R="
                 << right->q() << " m. Inspect whether the ball remains between the pads.\n";
            reported=true;
        }
        return true;
    }
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RealModelBallController)
