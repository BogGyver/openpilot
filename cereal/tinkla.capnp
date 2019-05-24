using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Java = import "./include/java.capnp";
$Java.package("ai.comma.openpilot.cereal");
$Java.outerClassname("Tinkla");

@0xfc8dda643156b95d;

struct Interface {

    message :union {
        userInfo @0: UserInfo;
        userEvent @1: UserEvent;
    }

    struct UserInfo {
        openPilotId @0 :Text;
        timestamp @1 :Text; # ISO8601

        userNickname @2 :Text;
        gitRemote @3 :Text;
        gitBranch @4 :Text;
    }

    struct UserEvent {
        openPilotId @0 :Text;
        timestamp @1 :Text;
        source @2 :Text;

        category @3 :Category;
        name @4 :Text;

        value :union {
            boolValue @5 :Bool;
            textValue @6 :Text;
            intValue @7 :Int64;
        }

        enum Category {
            general @0;
            userAction @1;
            openPilotAction @2;
            other @3;
        }
    }

}
