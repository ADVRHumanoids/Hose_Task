#include <XBotInterface/StateMachine.h>


namespace myfsm{

/*Example how to define a custom Event*/
/*  class MyEvent : public XBot::FSM::Event {

    public:

      MyEvent(int id): id(id) {}
      
      int id;    

    };
*/

/*Example how to define a custom Message*/   
/*  class MyMessage : public XBot::FSM::Message {

    public:
      
      MyMessage (int id):id(id){};
      
      int id;
	
    };
*/
    struct SharedData {
      
      XBot::RobotInterface::Ptr _robot;
     
    };
    
    class MacroState : public  XBot::FSM::State< MacroState , SharedData > {
      
    public:
	
	virtual void entry(const XBot::FSM::Message& msg) {};
	virtual void react(const XBot::FSM::Event& e){};
      
    };  

 
    class Home : public MacroState {

      virtual std::string get_name() const { return "Home"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Move_RH : public MacroState {

      virtual std::string get_name() const { return "Move_RH"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Grasp_RH : public MacroState {

      virtual std::string get_name() const { return "Grasp_RH"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Grasp_RH_Done : public MacroState {

      virtual std::string get_name() const { return "Grasp_RH_Done"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Orient_RH : public MacroState {

      virtual std::string get_name() const { return "Orient_RH"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Orient_RH_Done : public MacroState {

      virtual std::string get_name() const { return "Orient_RH_Done"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Move_LH : public MacroState {

      virtual std::string get_name() const { return "Move_LH"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Push_LH : public MacroState {

      virtual std::string get_name() const { return "Push_LH"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Push_LH_Done : public MacroState {

      virtual std::string get_name() const { return "Push_LH_Done"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Grasp_Fail : public MacroState {

      virtual std::string get_name() const { return "Grasp_Fail"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Orient_Fail : public MacroState {

      virtual std::string get_name() const { return "Orient_Fail"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Ungrasped_Done : public MacroState {

      virtual std::string get_name() const { return "Ungrasped_Done"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
 
    class Push_Fail : public MacroState {

      virtual std::string get_name() const { return "Push_Fail"; }

      virtual void run(double time, double period);

      virtual void entry(const XBot::FSM::Message& msg);

      virtual void react(const XBot::FSM::Event& e);

      virtual void exit ();

      private:


     };
     
      
}
