Unity Version 0.9.1-1 (2023-2-25)
========================
 * Bring Unity to OP 0.9.1

Version 0.9.2 (2023-03-XX)
========================
* Draw MPC path instead of model predicted path, this is a more accurate representation of what the car will do.

Version 0.9.1 (2023-02-23)
========================
* New driving model
  * 30% improved height estimation resulting in better driving performance for tall cars
* Driver monitoring: removed timer resetting on user interaction if distracted
* UI updates
  * Adjust alert volume using ambient noise level
  * Driver monitoring icon shows driver's head pose
  * German translation thanks to Vrabetz and CzokNorris!
* Cadillac Escalade 2017 support thanks to rickygilleland!
* Chevrolet Bolt EV 2022-23 support thanks to JasonJShuler!
* Genesis GV60 2023 support thanks to sunnyhaibin!
* Hyundai Tucson 2022-23 support
* Kia K5 Hybrid 2020 support thanks to sunnyhaibin!
* Kia Niro Hybrid 2023 support thanks to sunnyhaibin!
* Kia Sorento 2022-23 support thanks to sunnyhaibin!
* Kia Sorento Plug-in Hybrid 2022 support thanks to sunnyhaibin!
* Toyota C-HR 2021 support thanks to eFiniLan!
* Toyota C-HR Hybrid 2022 support thanks to Korben00!
* Volkswagen Crafter and MAN TGE 2017-23 support thanks to jyoung8607!

Version 0.9.0 (2022-11-21)
========================
* New driving model
  * Internal feature space information content increased tenfold during training to ~700 bits, which makes the model dramatically more accurate
  * Less reliance on previous frames makes model more reactive and snappy
  * Trained in new reprojective simulator
  * Trained in 36 hours from scratch, compared to one week for previous releases
  * Training now simulates both lateral and longitudinal behavior, which allows openpilot to slow down for turns, stop at traffic lights, and more in experimental mode
* Experimental driving mode
  * End-to-end longitudinal control
  * Stops for traffic lights and stop signs
  * Slows down for turns
  * openpilot defaults to chill mode, enable experimental mode in settings
* Driver monitoring updates
  * New bigger model with added end-to-end distracted trigger
  * Reduced false positives during driver calibration
* Self-tuning torque controller: learns parameters live for each car
* Torque controller used on all Toyota, Lexus, Hyundai, Kia, and Genesis models
* UI updates
  * Matched speeds shown on car's dash
  * Multi-language in navigation
  * Improved update experience
  * Border turns grey while overriding steering
  * Bookmark events while driving; view them in comma connect
  * New onroad visualization for experimental mode
* tools: new and improved cabana thanks to deanlee!
* Experimental longitudinal support for Volkswagen, CAN-FD Hyundai, and new GM models
* Genesis GV70 2022-23 support thanks to zunichky and sunnyhaibin!
* Hyundai Santa Cruz 2021-22 support thanks to sunnyhaibin!
* Kia Sportage 2023 support thanks to sunnyhaibin!
* Kia Sportage Hybrid 2023 support thanks to sunnyhaibin!
* Kia Stinger 2022 support thanks to sunnyhaibin!

Version 0.8.16 (2022-08-26)
========================
* New driving model
  * Reduced turn cutting
* Auto-detect right hand drive setting with driver monitoring model
* Improved fan controller for comma three
* New translations
  * Japanese thanks to cydia2020!
  * Brazilian Portuguese thanks to AlexandreSato!
* Chevrolet Bolt EUV 2022-23 support thanks to JasonJShuler!
* Chevrolet Silverado 1500 2020-21 support thanks to JasonJShuler!
* GMC Sierra 1500 2020-21 support thanks to JasonJShuler!
* Hyundai Ioniq 5 2022 support thanks to sunnyhaibin!
* Hyundai Kona Electric 2022 support thanks to sunnyhaibin!
* Hyundai Tucson Hybrid 2022 support thanks to sunnyhaibin!
* Subaru Legacy 2020-22 support thanks to martinl!
* Subaru Outback 2020-22 support

Version 0.8.15 (2022-07-20)
========================
* New driving model
  * Path planning uses end-to-end output instead of lane lines at all times
  * Reduced ping pong
  * Improved lane centering
* New lateral controller based on physical wheel torque model
  * Much smoother control that's consistent across the speed range
  * Effective feedforward that uses road roll
  * Simplified tuning, all car-specific parameters can be derived from data
  * Used on select Toyota and Hyundai models at first
  * Significantly improved control on TSS-P Prius
* New driver monitoring model
  * Bigger model, covering full interior view from driver camera
  * Works with a wider variety of mounting angles
  * 3x more unique comma three training data than previous
* Navigation improvements
  * Speed limits shown while navigating
  * Faster position fix by using raw GPS measurements
* UI updates
  * Multilanguage support for settings and home screen
  * New font
  * Refreshed max speed design
  * More consistent camera view perspective across cars
* Reduced power usage: device runs cooler and fan spins less
* AGNOS 5
  * Support VSCode remote SSH target
  * Support for delta updates to reduce data usage on future OS updates
* Chrysler ECU firmware fingerprinting thanks to realfast!
* Honda Civic 2022 support
* Hyundai Tucson 2021 support thanks to bluesforte!
* Kia EV6 2022 support
* Lexus NX Hybrid 2020 support thanks to AlexandreSato!
* Ram 1500 2019-21 support thanks to realfast!

Version 0.8.14 (2022-06-01)
========================
 * New driving model
   * Bigger model, using both of comma three's road-facing cameras
   * Better at cut-in detection and tight turns
 * New driver monitoring model
   * Tweaked network structure to improve output resolution for DSP
   * Fixed bug in quantization aware training to reduce quantizing errors
   * Resulted in 7x less MSE and no more random biases at runtime
 * Added toggle to disable disengaging on the accelerator pedal
 * comma body support
 * Audi RS3 support thanks to jyoung8607!
 * Hyundai Ioniq Plug-in Hybrid 2019 support thanks to sunnyhaibin!
 * Hyundai Tucson Diesel 2019 support thanks to sunnyhaibin!
 * Toyota Alphard Hybrid 2021 support
 * Toyota Avalon Hybrid 2022 support
 * Toyota RAV4 2022 support
 * Toyota RAV4 Hybrid 2022 support

Version 0.8.13 (2022-02-18)
========================
 * Improved driver monitoring
   * Re-tuned driver pose learner for relaxed driving positions
   * Added reliance on driving model to be more scene adaptive
   * Matched strictness between comma two and comma three
 * Improved performance in turns by compensating for the road bank angle
 * Improved camera focus on the comma two
 * AGNOS 4
   * ADB support
   * improved cell auto configuration
 * NEOS 19
   * package updates
   * stability improvements
 * Subaru ECU firmware fingerprinting thanks to martinl!
 * Hyundai Santa Fe Plug-in Hybrid 2022 support thanks to sunnyhaibin!
 * Mazda CX-5 2022 support thanks to Jafaral!
 * Subaru Impreza 2020 support thanks to martinl!
 * Toyota Avalon 2022 support thanks to sshane!
 * Toyota Prius v 2017 support thanks to CT921!
 * Volkswagen Caravelle 2020 support thanks to jyoung8607!

Version 0.8.12 (2021-12-15)
========================
 * New driving model
   * Improved behavior around exits
   * Better pose accuracy at high speeds, allowing max speed of 90mph
   * Fully incorporated comma three data into all parts of training stack
 * Improved follow distance
 * Better longitudinal policy, especially in low speed traffic
 * New alert sounds
 * AGNOS 3
   * Display burn in mitigation
   * Improved audio amplifier configuration
   * System reliability improvements
   * Update Python to 3.8.10
 * Raw logs upload moved to connect.comma.ai
 * Fixed HUD alerts on newer Honda Bosch thanks to csouers!
 * Audi Q3 2020-21 support thanks to jyoung8607!
 * Lexus RC 2020 support thanks to ErichMoraga!

Version 0.8.11 (2021-11-29)
========================
 * Support for CAN FD on the red panda
 * Support for an external panda on the comma three
 * Navigation: Show more detailed instructions when approaching maneuver
 * Fixed occasional steering faults on GM cars thanks to jyoung8607!
 * Nissan ECU firmware fingerprinting thanks to robin-reckmann, martinl, and razem-io!
 * Cadillac Escalade ESV 2016 support thanks to Gibby!
 * Genesis G70 2020 support thanks to tecandrew!
 * Hyundai Santa Fe Hybrid 2022 support thanks to sunnyhaibin!
 * Mazda CX-9 2021 support thanks to Jacar!
 * Volkswagen Polo 2020 support thanks to jyoung8607!
 * Volkswagen T-Roc 2021 support thanks to jyoung8607!

Version 0.8.10 (2021-11-01)
========================
 * New driving model
   * Trained on one million minutes!!!
   * Fixed lead training making lead predictions significantly more accurate
   * Fixed several localizer dataset bugs and loss function bugs, overall improved accuracy
 * New driver monitoring model
   * Trained on latest data from both comma two and comma three
   * Increased model field of view by 40% on comma three
   * Improved model stability on masked users
   * Improved pose prediction with reworked ground-truth stack
 * Lateral and longitudinal planning MPCs now in ACADOS
 * Combined longitudinal MPCs
   * All longitudinal planning now happens in a single MPC system
   * Fixed instability in MPC problem to prevent sporadic CPU usage
 * AGNOS 2: minor stability improvements and builder repo open sourced
 * tools: new and improved replay thanks to deanlee!
 * Moved community-supported cars outside of the Community Features toggle
 * Improved FW fingerprinting reliability for Hyundai/Kia/Genesis
 * Added prerequisites for longitudinal control on Hyundai/Kia/Genesis and Honda Bosch
 * Audi S3 2015 support thanks to jyoung8607!
 * Honda Freed 2020 support thanks to belm0!
 * Hyundai Ioniq Hybrid 2020-2022 support thanks to sunnyhaibin!
 * Hyundai Santa Fe 2022 support thanks to sunnyhaibin!
 * Kia K5 2021 support thanks to sunnyhaibin!
 * Škoda Kamiq 2021 support thanks to jyoung8607!
 * Škoda Karoq 2019 support thanks to jyoung8607!
 * Volkswagen Arteon 2021 support thanks to jyoung8607!
 * Volkswagen California 2021 support thanks to jyoung8607!
 * Volkswagen Taos 2022 support thanks to jyoung8607!

Version 0.8.9 (2021-09-14)
========================
 * Improved fan control on comma three
 * AGNOS 1.5: improved stability
 * Honda e 2020 support

Version 0.8.8 (2021-08-27)
========================
 * New driving model with improved laneless performance
   * Trained on 5000+ hours of diverse driving data from 3000+ users in 40+ countries
   * Better anti-cheating methods during simulator training ensure the model hugs less when in laneless mode
   * All new desire ground-truthing stack makes the model better at lane changes
 * New driver monitoring model: improved performance on comma three
 * NEOS 18 for comma two: update packages
 * AGNOS 1.3 for comma three: fix display init at high temperatures
 * Improved auto-exposure on comma three
 * Improved longitudinal control on Honda Nidec cars
 * Hyundai Kona Hybrid 2020 support thanks to haram-KONA!
 * Hyundai Sonata Hybrid 2021 support thanks to Matt-Wash-Burn!
 * Kia Niro Hybrid 2021 support thanks to tetious!

Version 0.8.7 (2021-07-31)
========================
 * comma three support!
 * Navigation alpha for the comma three!
 * Volkswagen T-Cross 2021 support thanks to jyoung8607!

Version 0.8.6 (2021-07-21)
========================
 * Revamp lateral and longitudinal planners
   * Refactor planner output API to be more readable and verbose
   * Planners now output desired trajectories for speed, acceleration, curvature, and curvature rate
   * Use MPC for longitudinal planning when no lead car is present, makes accel and decel smoother
 * Remove "CHECK DRIVER FACE VISIBILITY" warning
 * Fixed cruise fault on some TSS2.5 Camrys and international Toyotas
 * Hyundai Elantra Hybrid 2021 support thanks to tecandrew!
 * Hyundai Ioniq PHEV 2020 support thanks to YawWashout!
 * Kia Niro Hybrid 2019 support thanks to jyoung8607!
 * Škoda Octavia RS 2016 support thanks to jyoung8607!
 * Toyota Alphard 2020 support thanks to belm0!
 * Volkswagen Golf SportWagen 2015 support thanks to jona96!
 * Volkswagen Touran 2017 support thanks to jyoung8607!

Version 0.8.5 (2021-06-11)
========================
 * NEOS update: improved reliability and stability with better voltage regulator configuration
 * Smart model-based Forward Collision Warning
 * CAN-based fingerprinting moved behind community features toggle
 * Improved longitudinal control on Toyotas with a comma pedal
 * Improved auto-brightness using road-facing camera
 * Added "Software" settings page with updater controls
 * Audi Q2 2018 support thanks to jyoung8607!
 * Hyundai Elantra 2021 support thanks to CruiseBrantley!
 * Lexus UX Hybrid 2019-2020 support thanks to brianhaugen2!
 * Toyota Avalon Hybrid 2019 support thanks to jbates9011!
 * SEAT Leon 2017 & 2020 support thanks to jyoung8607!
 * Škoda Octavia 2015 & 2019 support thanks to jyoung8607!

Version 0.8.4 (2021-05-17)
========================
 * Delay controls start until system is ready
 * Fuzzy car identification, enabled with Community Features toggle
 * Localizer optimized for increased precision and less CPU usage
 * Re-tuned lateral control to be more aggressive when model is confident
 * Toyota Mirai 2021 support
 * Lexus NX 300 2020 support thanks to goesreallyfast!
 * Volkswagen Atlas 2018-19 support thanks to jyoung8607!

Version 0.8.3 (2021-04-01)
========================
 * New model
   * Trained on new diverse dataset from 2000+ users from 30+ countries
   * Trained with improved segnet from the comma-pencil community project
   * 🥬 Dramatically improved end-to-end lateral performance 🥬
 * Toggle added to disable the use of lanelines
 * NEOS update: update packages and support for new UI
 * New offroad UI based on Qt
 * Default SSH key only used for setup
 * Kia Ceed 2019 support thanks to ZanZaD13!
 * Kia Seltos 2021 support thanks to speedking456!
 * Added support for many Volkswagen and Škoda models thanks to jyoung8607!

Version 0.8.2 (2021-02-26)
========================
 * Use model points directly in MPC (no more polyfits), making lateral planning more accurate
 * Use model heading prediction for smoother lateral control
 * Smarter actuator delay compensation
 * Improve qcamera resolution for improved video in explorer and connect
 * Adjust maximum engagement speed to better fit the model's training distribution
 * New driver monitoring model trained with 3x more diverse data
 * Improved face detection with masks
 * More predictable DM alerts when visibility is bad
 * Rewritten video streaming between openpilot processes
 * Improved longitudinal tuning on TSS2 Corolla and Rav4 thanks to briskspirit!
 * Audi A3 2015 and 2017 support thanks to keeleysam!
 * Nissan Altima 2020 support thanks to avolmensky!
 * Lexus ES Hybrid 2018 support thanks to TheInventorMan!
 * Toyota Camry Hybrid 2021 support thanks to alancyau!

Version 0.8.1 (2020-12-21)
========================
 * Original EON is deprecated, upgrade to comma two
 * Better model performance in heavy rain
 * Better lane positioning in turns
 * Fixed bug where model would cut turns on empty roads at night
 * Fixed issue where some Toyotas would not completely stop thanks to briskspirit!
 * Toyota Camry 2021 with TSS2.5 support
 * Hyundai Ioniq Electric 2020 support thanks to baldwalker!

Version 0.8.0 (2020-11-30)
========================
 * New driving model: fully 3D and improved cut-in detection
 * UI draws 2 road edges, 4 lanelines and paths in 3D
 * Major fixes to cut-in detection for openpilot longitudinal
 * Grey panda is no longer supported, upgrade to comma two or black panda
 * Lexus NX 2018 support thanks to matt12eagles!
 * Kia Niro EV 2020 support thanks to nickn17!
 * Toyota Prius 2021 support thanks to rav4kumar!
 * Improved lane positioning with uncertain lanelines, wide lanes and exits
 * Improved lateral control for Prius and Subaru
