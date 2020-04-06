
WARNING: TESLA ONLY OPENPILOT 0.6.6-T14
======
This repo contains code that was modified specifically for Tesla and will not work on other cars!

Main Comma.ai code is Copyright (c) 2018, Comma.ai, Inc. Additonal work (ALCA, webcamera, any modifications to base Comma.ao code) is licensed under <a rel="license" href="http://creativecommons.org/licenses/by-nc-nd/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-nd/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-nd/4.0/">Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License</a>.

[![](https://i.imgur.com/UetIFyH.jpg)](#)

Welcome to openpilot
======

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, it performs the functions of Adaptive Cruise Control (ACC) and Automated Lane Centering (ALC) for selected Honda, Toyota, Acura, Lexus, Chevrolet, Hyundai, Kia, Subaru, Volkswagen. It's about on par with Tesla Autopilot and GM Super Cruise, and better than [all other manufacturers](http://www.thedrive.com/tech/5707/the-war-for-autonomous-driving-part-iii-us-vs-germany-vs-japan).

The openpilot codebase has been written to be concise and to enable rapid prototyping. We look forward to your contributions - improving real vehicle automation has never been easier.

Table of Contents
=======================

* [Community](#community)
* [Hardware](#hardware)
* [Supported Cars](#supported-cars)
* [Community Maintained Cars](#community-maintained-cars)
* [In Progress Cars](#in-progress-cars)
* [How can I add support for my car?](#how-can-i-add-support-for-my-car)
* [Directory structure](#directory-structure)
* [User Data / chffr Account / Crash Reporting](#user-data--chffr-account--crash-reporting)
* [Testing on PC](#testing-on-pc)
* [Contributing](#contributing)
* [Licensing](#licensing)

---

Community
------

openpilot is developed by [comma.ai](https://comma.ai/) and users like you.

[Follow us on Twitter](https://twitter.com/comma_ai) and [join our Discord](https://discord.comma.ai).

<table>
  <tr>
    <td><a href="https://www.youtube.com/watch?v=mgAbfr42oI8" title="YouTube" rel="noopener"><img src="https://i.imgur.com/kAtT6Ei.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=394rJKeh76k" title="YouTube" rel="noopener"><img src="https://i.imgur.com/lTt8cS2.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=1iNOc3cq8cs" title="YouTube" rel="noopener"><img src="https://i.imgur.com/ANnuSpe.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=Vr6NgrB-zHw" title="YouTube" rel="noopener"><img src="https://i.imgur.com/Qypanuq.png"></a></td>
  </tr>
  <tr>
    <td><a href="https://www.youtube.com/watch?v=Ug41KIKF0oo" title="YouTube" rel="noopener"><img src="https://i.imgur.com/3caZ7xM.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=NVR_CdG1FRg" title="YouTube" rel="noopener"><img src="https://i.imgur.com/bAZOwql.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=tkEvIdzdfUE" title="YouTube" rel="noopener"><img src="https://i.imgur.com/EFINEzG.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=_P-N1ewNne4" title="YouTube" rel="noopener"><img src="https://i.imgur.com/gAyAq22.png"></a></td>
  </tr>
</table>

Hardware
------

At the moment openpilot supports the [EON DevKit](https://comma.ai/shop/products/eon-dashcam-devkit). A [car harness](https://comma.ai/shop/products/car-harness) is recommended to connect the EON to the car. We'd like to support other platforms as well.

Install openpilot on a neo device by entering ``https://openpilot.comma.ai`` during NEOS setup.

Supported Cars
------

| Make                  | Model (US Market Reference)        | Supported Package    | Lateral | Longitudinal   | No Accel Below   | No Steer Below |
| ----------------------| -----------------------------------| ---------------------| --------| ---------------| -----------------| ---------------|
| Acura                 | ILX 2016-18                        | AcuraWatch Plus      | Yes     | Yes            | 25mph<sup>1</sup>| 25mph          |
| Acura                 | RDX 2016-18                        | AcuraWatch Plus      | Yes     | Yes            | 25mph<sup>1</sup>| 12mph          |
| Buick<sup>3</sup>     | Regal 2018<sup>6</sup>             | Adaptive Cruise      | Yes     | Yes            | 0mph             | 7mph           |
| Chevrolet<sup>3</sup> | Malibu 2017<sup>6</sup>            | Adaptive Cruise      | Yes     | Yes            | 0mph             | 7mph           |
| Chevrolet<sup>3</sup> | Volt 2017-18<sup>6</sup>           | Adaptive Cruise      | Yes     | Yes            | 0mph             | 7mph           |
| Cadillac<sup>3</sup>  | ATS 2018                           | Adaptive Cruise      | Yes     | Yes            | 0mph             | 7mph           |
| Chrysler              | Pacifica 2017-18<sup>7</sup>       | Adaptive Cruise      | Yes     | Stock          | 0mph             | 9mph           |
| Chrysler              | Pacifica Hybrid 2017-18<sup>7</sup>| Adaptive Cruise      | Yes     | Stock          | 0mph             | 9mph           |
| Chrysler              | Pacifica Hybrid 2019<sup>7</sup>   | Adaptive Cruise      | Yes     | Stock          | 0mph             | 39mph          |
| GMC<sup>3</sup>       | Acadia Denali 2018<sup>6</sup>     | Adaptive Cruise      | Yes     | Yes            | 0mph             | 7mph           |
| Holden<sup>3</sup>    | Astra 2017<sup>6</sup>             | Adaptive Cruise      | Yes     | Yes            | 0mph             | 7mph           |
| Honda                 | Accord 2018-19                     | All                  | Yes     | Stock          | 0mph             | 3mph           |
| Honda                 | Accord Hybrid 2018-19              | All                  | Yes     | Stock          | 0mph             | 3mph           |
| Honda                 | Civic Sedan/Coupe 2016-18          | Honda Sensing        | Yes     | Yes            | 0mph             | 12mph          |
| Honda                 | Civic Sedan/Coupe 2019             | Honda Sensing        | Yes     | Stock          | 0mph             | 2mph           |
| Honda                 | Civic Hatchback 2017-19            | Honda Sensing        | Yes     | Stock          | 0mph             | 12mph          |
| Honda                 | CR-V 2015-16                       | Touring              | Yes     | Yes            | 25mph<sup>1</sup>| 12mph          |
| Honda                 | CR-V 2017-19                       | Honda Sensing        | Yes     | Stock          | 0mph             | 12mph          |
| Honda                 | CR-V Hybrid 2017-2019              | Honda Sensing        | Yes     | Stock          | 0mph             | 12mph          |
| Honda                 | Fit 2018-19                        | Honda Sensing        | Yes     | Yes            | 25mph<sup>1</sup>| 12mph          |
| Honda                 | Odyssey 2018-19                    | Honda Sensing        | Yes     | Yes            | 25mph<sup>1</sup>| 0mph           |
| Honda                 | Passport 2019                      | All                  | Yes     | Yes            | 25mph<sup>1</sup>| 12mph          |
| Honda                 | Pilot 2016-18                      | Honda Sensing        | Yes     | Yes            | 25mph<sup>1</sup>| 12mph          |
| Honda                 | Pilot 2019                         | All                  | Yes     | Yes            | 25mph<sup>1</sup>| 12mph          |
| Honda                 | Ridgeline 2017-19                  | Honda Sensing        | Yes     | Yes            | 25mph<sup>1</sup>| 12mph          |
| Hyundai               | Santa Fe 2019<sup>5</sup>          | All                  | Yes     | Stock          | 0mph             | 0mph           |
| Hyundai               | Elantra 2017-19<sup>5</sup>        | SCC + LKAS           | Yes     | Stock          | 19mph            | 34mph          |
| Hyundai               | Genesis 2018<sup>5</sup>           | All                  | Yes     | Stock          | 19mph            | 34mph          |
| Jeep                  | Grand Cherokee 2016-18<sup>7</sup> | Adaptive Cruise      | Yes     | Stock          | 0mph             | 9mph           |
| Jeep                  | Grand Cherokee 2019<sup>7</sup>    | Adaptive Cruise      | Yes     | Stock          | 0mph             | 39mph          |
| Kia                   | Optima 2019<sup>5</sup>            | SCC + LKAS           | Yes     | Stock          | 0mph             | 0mph           |
| Kia                   | Sorento 2018<sup>5</sup>           | All                  | Yes     | Stock          | 0mph             | 0mph           |
| Kia                   | Stinger 2018<sup>5</sup>           | SCC + LKAS           | Yes     | Stock          | 0mph             | 0mph           |
| Lexus                 | CT Hybrid 2017-18                  | All                  | Yes     | Yes<sup>2</sup>| 0mph             | 0mph           |
| Lexus                 | ES Hybrid 2019                     | All                  | Yes     | Yes            | 0mph             | 0mph           |
| Lexus                 | RX Hybrid 2016-19                  | All                  | Yes     | Yes<sup>2</sup>| 0mph             | 0mph           |
| Lexus                 | IS 2017-2019                       | All                  | Yes     | Stock          | 22mph            | 0mph           |
| Lexus                 | IS Hybrid 2017                     | All                  | Yes     | Stock          | 0mph             | 0mph           |
| Subaru                | Crosstrek 2018-19                  | EyeSight             | Yes     | Stock          | 0mph             | 0mph           |
| Subaru                | Impreza 2019-20                    | EyeSight             | Yes     | Stock          | 0mph             | 0mph           |
| Toyota                | Avalon 2016                        | TSS-P                | Yes     | Yes<sup>2</sup>| 20mph<sup>1</sup>| 0mph           |
| Toyota                | Avalon 2017-18                     | All                  | Yes     | Yes<sup>2</sup>| 20mph<sup>1</sup>| 0mph           |
| Toyota                | Camry 2018-19                      | All                  | Yes     | Stock          | 0mph<sup>4</sup> | 0mph           |
| Toyota                | Camry Hybrid 2018-19               | All                  | Yes     | Stock          | 0mph<sup>4</sup> | 0mph           |
| Toyota                | C-HR 2017-19                       | All                  | Yes     | Stock          | 0mph             | 0mph           |
| Toyota                | C-HR Hybrid 2017-19                | All                  | Yes     | Stock          | 0mph             | 0mph           |
| Toyota                | Corolla 2017-19                    | All                  | Yes     | Yes<sup>2</sup>| 20mph<sup>1</sup>| 0mph           |
| Toyota                | Corolla 2020                       | All                  | Yes     | Yes            | 0mph             | 0mph           |
| Toyota                | Corolla Hatchback 2019             | All                  | Yes     | Yes            | 0mph             | 0mph           |
| Toyota                | Corolla Hybrid 2020                | All                  | Yes     | Yes            | 0mph             | 0mph           |
| Toyota                | Highlander 2017-19                 | All                  | Yes     | Yes<sup>2</sup>| 0mph             | 0mph           |
| Toyota                | Highlander Hybrid 2017-19          | All                  | Yes     | Yes<sup>2</sup>| 0mph             | 0mph           |
| Toyota                | Prius 2016                         | TSS-P                | Yes     | Yes<sup>2</sup>| 0mph             | 0mph           |
| Toyota                | Prius 2017-19                      | All                  | Yes     | Yes<sup>2</sup>| 0mph             | 0mph           |
| Toyota                | Prius Prime 2017-20                | All                  | Yes     | Yes<sup>2</sup>| 0mph             | 0mph           |
| Toyota                | Rav4 2016                          | TSS-P                | Yes     | Yes<sup>2</sup>| 20mph<sup>1</sup>| 0mph           |
| Toyota                | Rav4 2017-18                       | All                  | Yes     | Yes<sup>2</sup>| 20mph<sup>1</sup>| 0mph           |
| Toyota                | Rav4 2019                          | All                  | Yes     | Yes            | 0mph             | 0mph           |
| Toyota                | Rav4 Hybrid 2016                   | TSS-P                | Yes     | Yes<sup>2</sup>| 0mph             | 0mph           |
| Toyota                | Rav4 Hybrid 2017-18                | All                  | Yes     | Yes<sup>2</sup>| 0mph             | 0mph           |
| Toyota                | Sienna 2018                        | All                  | Yes     | Yes<sup>2</sup>| 0mph             | 0mph           |
| Volkswagen<sup>8</sup>| Golf 2016-19                       | Driver Assistance    | Yes     | Stock          | 0mph             | 0mph           |

<sup>1</sup>[Comma Pedal](https://community.comma.ai/wiki/index.php/Comma_Pedal) is used to provide stop-and-go capability to some of the openpilot-supported cars that don't currently support stop-and-go. Here is how to [build a Comma Pedal](https://medium.com/@jfrux/comma-pedal-building-with-macrofab-6328bea791e8). ***NOTE: The Comma Pedal is not officially supported by [comma.ai](https://comma.ai).*** <br />
<sup>2</sup>When disconnecting the Driver Support Unit (DSU), otherwise longitudinal control is stock ACC. For DSU locations, see [Toyota Wiki page](https://community.comma.ai/wiki/index.php/Toyota). ***NOTE: disconnecting the DSU disables Automatic Emergency Braking (AEB).*** <br />
<sup>3</sup>[GM installation guide](https://zoneos.com/volt/). ***NOTE: disconnecting the ASCM disables Automatic Emergency Braking (AEB).*** <br />
<sup>4</sup>28mph for Camry 4CYL L, 4CYL LE and 4CYL SE which don't have Full-Speed Range Dynamic Radar Cruise Control. <br />
<sup>5</sup>Requires a [panda](https://comma.ai/shop/products/panda-obd-ii-dongle) and open sourced [Hyundai Giraffe](https://github.com/commaai/neo/tree/master/giraffe/hyundai), designed for the 2019 Sante Fe; pinout may differ for other Hyundais. <br />
<sup>6</sup>Requires a [panda](https://comma.ai/shop/products/panda-obd-ii-dongle) and community built giraffe, find more information [here](https://zoneos.com/shop/). <br />
<sup>7</sup>Requires a [panda](https://comma.ai/shop/products/panda-obd-ii-dongle) and FCA [giraffe](https://comma.ai/shop/products/giraffe) <br />
<sup>8</sup>Requires a [custom connector](https://community.comma.ai/wiki/index.php/Volkswagen#Integration_at_J533_Gateway) for the [car harness](https://comma.ai/shop/products/car-harness) <br />

Community Maintained Cars
------

| Make                 | Model (US Market Reference)        | Supported Package    | Lateral | Longitudinal   | No Accel Below   | No Steer Below |
| ---------------------| -----------------------------------| ---------------------| --------| ---------------| -----------------| ---------------|
| Tesla                | Model S 2012-13<sup>9</sup>        | All                  | Yes     | NA             | NA               | 0mph           |

[[Tesla Model S Pull Request]](https://github.com/commaai/openpilot/pull/246) <br />
<sup>9</sup>Requires a [panda](https://comma.ai/shop/products/panda-obd-ii-dongle) and community built giraffe, find more information [here](https://github.com/jeankalud/neo/tree/tesla_giraffe/giraffe/tesla). <br />

Community Maintained Cars are not confirmed by comma.ai to meet our [safety model](https://github.com/commaai/openpilot/blob/devel/SAFETY.md). Be extra cautious using them.

In Progress Cars
------
- All TSS-P Toyota with Steering Assist and LSS-P Lexus with Steering Assist or Lane Keep Assist.
- All Hyundai with SmartSense.
- All Kia, Genesis with SCC and LKAS.
- All Chrysler, Jeep, Fiat with Adaptive Cruise Control and LaneSense.
- All Subaru with EyeSight.
- All Volkswagen, Audi, Škoda and SEAT with Adaptive Cruise Control.

How can I add support for my car?
------

If your car has adaptive cruise control and lane keep assist, you are in luck. Using a [panda](https://comma.ai/shop/products/panda-obd-ii-dongle/) and [cabana](https://community.comma.ai/cabana/), you can understand how to make your car drive by wire.

We've written guides for [Brand](https://medium.com/@comma_ai/how-to-write-a-car-port-for-openpilot-7ce0785eda84) and [Model](https://medium.com/@comma_ai/openpilot-port-guide-for-toyota-models-e5467f4b5fe6) ports. These guides might help you after you have the basics figured out.

- BMW, Audi, Volvo, and Mercedes all use [FlexRay](https://en.wikipedia.org/wiki/FlexRay) and can be supported after [FlexRay support](https://github.com/commaai/openpilot/pull/463) is merged.
- We put time into a Ford port, but the steering has a 10 second cutout limitation that makes it unusable.
- The 2016-2017 Honda Accord uses a custom signaling protocol for steering that's unlikely to ever be upstreamed.

Directory structure
------
    .
    ├── apk                 # The apk files used for the UI
    ├── cereal              # The messaging spec used for all logs on EON
    ├── common              # Library like functionality we've developed here
    ├── installer/updater   # Manages auto-updates of openpilot
    ├── opendbc             # Files showing how to interpret data from cars
    ├── panda               # Code used to communicate on CAN and LIN
    ├── phonelibs           # Libraries used on EON
    ├── pyextra             # Libraries used on EON
    └── selfdrive           # Code needed to drive the car
        ├── assets          # Fonts and images for UI
        ├── athena          # Allows communication with the app
        ├── boardd          # Daemon to talk to the board
        ├── can             # Helpers for parsing CAN messages
        ├── car             # Car specific code to read states and control actuators
        ├── common          # Shared C/C++ code for the daemons
        ├── controls        # Perception, planning and controls
        ├── debug           # Tools to help you debug and do car ports
        ├── locationd       # Soon to be home of precise location
        ├── logcatd         # Android logcat as a service
        ├── loggerd         # Logger and uploader of car data
        ├── proclogd        # Logs information from proc
        ├── sensord         # IMU / GPS interface code
        ├── test            # Car simulator running code through virtual maneuvers
        ├── ui              # The UI
        └── visiond         # Vision pipeline

To understand how the services interact, see `selfdrive/service_list.yaml`

User Data / chffr Account / Crash Reporting
------

By default, openpilot creates an account and includes a client for chffr, our dashcam app. We use your data to train better models and improve openpilot for everyone.

It's open source software, so you are free to disable it if you wish.

It logs the road facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The user facing camera is only logged if you explicitly opt-in in settings.
It does not log the microphone.

By using it, you agree to [our privacy policy](https://my.comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma.ai. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma.ai for the use of this data.

Testing on PC
------

Check out [openpilot-tools](https://github.com/commaai/openpilot-tools): lots of tools you can use to replay driving data, test and develop openpilot from your pc.

Also, within openpilot there is a rudimentary infrastructure to run a basic simulation and generate a report of openpilot's behavior in different longitudinal control scenarios.

```bash
# Requires working docker
./run_docker_tests.sh
```

Contributing
------

We welcome both pull requests and issues on [github](http://github.com/commaai/openpilot). Bug fixes and new car ports encouraged.

We also have a [bounty program](https://comma.ai/bounties.html).

Want to get paid to work on openpilot? [comma.ai is hiring](https://comma.ai/jobs/)

Licensing
------

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>
