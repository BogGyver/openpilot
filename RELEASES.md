Unity Version 0.8.13-56 (2022-10-27)
========================
 * Sound files volume fix
 * Show TACC icon on IC for AP cars when OP is available
 * Display shutdown
 * Engage at 0 MPH (AP cars)
 * Fleet based speed (AP cars)
 * Slow down in turns based on SunnyPilot 
 * prevent iBooster from pressing both pedals
 * improved acceleration handling for AP1
 * Add message ID for the CAN Error 
 * Reduce min accel to -4.5 
 * Improve follow distance
 * Better and smoother pedal with 4 proiles

Unity Version 0.8.13-55 (2022-09-21)
========================
 * Sound toggles (to mute certain sounds)
 * Sound files

Unity Version 0.8.13-54 (2022-09-19)
========================
 * Add capabilities for Str params
 * Implement fixed fingerprint option to avoid fingerprinting issues

Unity Version 0.8.13-53 (2022-08-05)
========================
 * iBooster ECU fixes
 * Try to fix Controls Mismatch issues
 * Add toggle for radar error with AP
 
 Unity Version 0.8.13-52 (2022-08-05)
========================
 * Reset Pedal PID on engagement
 * Toggle for Model S Performance for pedal (bug fix)
 * Pedal profile for MS Performance (bug fix)
 * Toggle to prevent auto updates
 * Toggle for dev unit (bug fix)
 * Autoresume speed from stand still

Unity Version 0.8.13-51 (2022-07-05)
========================
 * New PID for pedal, including way to save state
 * Toggle for Model S Performance for pedal
 * Pedal profile for MS Performance

Unity Version 0.8.13-49 (2022-05-30)
========================
 * Fix CAN-flashing code
 
 Unity Version 0.8.13-48 (2022-05-18)
========================
 * Limit iBooster travel to a max of 15mm (90psi on my car)
 * Set Hold values for iBooster to 6.5mm (14psi)
 * Change lane poly calculation logic for IC integrtion

Unity Version 0.8.13-47 (2022-05-11)
========================
 * Do not send iBooster brake command when real accelerator pedal is pressed

Unity Version 0.8.13-46 (2022-05-10)
========================
 * Do not send iBooster brake command when accelerator pedal is pressed
 * Increase brake hold value 

Unity Version 0.8.13-45 (2022-05-09s)
========================
 * Update firmware for Vacuum Sensor board with brake release condition
 * Allow 0x553 for iBooster in panda
 * Allow 0x555 for IVS in panda
 
 Unity Version 0.8.13-44 (2022-05-04)
========================
 * Update firmware for Vacuum Sensor board
 * Fix CRC for iBooster

Unity Version 0.8.13-43 (2022-04-21)
========================
 * Fix Pedal over CC issues when MCU2 or no Tinkla Buddy
 * Add firmware for Vacuum Sensor board
 * Fix CRC for iBooster

Unity Version 0.8.13-41 (2022-04-19)
========================
 * Fix startup screen for C3
 * Fix modem initialization for C3 (comma three: correctly set initial EPS bearer settings for AT&T sim cards)
 * Fix logic for enabling OP on preAP MS
 * Add C3 Tinkla Splash

Unity Version 0.8.13-39 (2022-04-16)
========================
 * Fix bug in iBooster ECU messaging

Unity Version 0.8.13-38 (2022-04-05)
========================
 * Always show OP data on IC for AP1/AP2
 * Toggle to switch the maps on the left side of the screen (Comma three)
 * fix speed limit indicator when using pedal with Tinkla Buddy

Unity Version 0.8.13-37 (2022-04-01)
========================
 * added configuration value for HandsOnLevel before human takeover
 * fixed acceleration for AP1 stop-and-go which was limited at 1.2m/s^2
 * added testing toggle for full LongControl from planner vs actuator (AP1 or preAP with iBooster)
 * fixed release scripts
 * fixed automatic flashing of panda code

Unity Version 0.8.13-34 (2022-03-27)
========================
 * full support for OP long control with AP1
 * follow distance is adjustable from CC stalk (when available)
 * UI shows multiple road lanes when detected by OP
 * fix bug where ACC would show disabled message after brake press even when not enabled
 * variable acceleration based on speed for AP1 OP based long control
 * improve pedal tune (by vandreykiv)
 * fix variables default value not showing correctly in UI

Unity Version 0.8.13-33 (2022-03-26)
========================
 * new events to show when Standard CC is enabled
 * added shutdown timeout for device (vandreykiv)

Unity Version 0.8.13-32 (2022-03-20)
========================
 * allows Standard CC (no LKAS) to be used with pedal when cruise enabled
 * allows Standard CC (no LKAS) to be used with ACC by double press down
 * allows for pedal to be used over CC (when setting enabled)

Unity Version 0.8.13-31 (2022-03-18)
========================
 * requires EON Gold/Black Panda or Comma two/three
 * no need for EPAS harness
 * Panda flash built in UI
 * Pedal flash built in UI
 * EPAS patching built in UI
 * Radar VIN Learner built in UI
 * radar behind nosecone setting as part of VIN Learn (set via UI)
 * works with either human long control, ACC or pedal
 * no more ssh to install or update
 * no more ssh to change any settings, all done through UI
 * automatic change top speed based on speed limit and offset (either units or %)
 * automatic lane change with adjustable delay
 * human steering override with adjustable delay for re-engagement
 * steering never disengages unless you cancel it via stalk
 * adjustable follow distance though UI
 * allows for CC without LKAS when using Pedal
 * pedal can be on either CAN0 or CAN2 (set via UI)
 * support for iBooster ECU (in dev) 


Version 0.8.13 (2022-02-18)
========================
 * Improved driver monitoring
   * Retuned driver pose learner for relaxed driving positions
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
 * Retuned lateral control to be more aggressive when model is confident
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
