using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Java = import "./include/java.capnp";
$Java.package("ai.comma.openpilot.cereal");
$Java.outerClassname("Ui");

using Car = import "car.capnp";

@0xce6ca45dddcd5317;

struct UIButtonInfo {
    # button ID 0..5
    btnId @0 :Int8;
    # internal button name
    btnName @1 :Text;
    # display label for button (3 chars)
    btnLabel @2 :Text;
    # buttons status: 0 = DISABLED, 1 = AVAILABLE, 2 = ENABLED, 3 = WARNING, 9 = NOT AVAILABLE
    btnStatus @3 :Int16;
    # small font label shows below the main label, max 7 chars
    btnLabel2 @4 :Text;
}

struct UIButtonStatus {
    # button ID 0..5
    btnId @0 :Int8;
    # buttons status: 0 = DISABLED, 1 = AVAILABLE, 2 = ENABLED, 3 = WARNING, 9 = NOT AVAILABLE
    btnStatus @1 :Int16;
}

struct UICustomAlert {
    caStatus @0 :Int8;
    caText @1 :Text;
}

struct UISetCar {
    icCarFolder @0 :Text;
    icCarName @1 :Text;
    icShowLogo @2 :Int8;
    icShowCar @3 :Int8;
}

struct UIPlaySound {
    sndSound @0 :Int8;
}

struct UIUpdate {
    uiDoUpdate @0 :Int8;
    uiStatus @1 :Int8;
    uiCanDisplayMessage @2 :Int8;
}

struct UIGyroInfo {
    accPitch @0 :Float32;
    accRoll @1 :Float32;
    accYaw @2 :Float32;
    magPitch @3 :Float32;
    magRoll @4 :Float32;
    magYaw @5 :Float32;
    gyroPitch @6 :Float32;
    gyroRoll @7 :Float32;
    gyroYaw @8 :Float32;
}




