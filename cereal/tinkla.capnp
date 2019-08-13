using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Java = import "./include/java.capnp";
$Java.package("ai.comma.openpilot.cereal");
$Java.outerClassname("Tinkla");

@0xfc8dda643156b95d;

const interfaceVersion :Float32 = 2.1;

struct Interface {

    version @0 :Float32;

    message :union {
        userInfo @1 :UserInfo;
        event @2 :Event;
        action @3 :Text;
    }

    struct UserInfo {
        version @0 :Float32;

        openPilotId @1 :Text;
        timestamp @2 :Text;

        userHandle @3 :Text;
        gitRemote @4 :Text;
        gitBranch @5 :Text;
        gitHash @6 :Text;
    }

    struct Event {
        version @0 :Float32;

        openPilotId @1 :Text;
        timestamp @2 :Text;
        source @3 :Text;

        category @4 :Category;
        name @5 :Text;

        value :union {
            boolValue @6 :Bool;
            textValue @7 :Text;
            intValue @8 :Int64;
            floatValue @9 :Float64;
        }

        enum Category {
            general @0;
            userAction @1;
            openPilotAction @2;
            crash @3;
            other @4;
        }
    }

}
