import configparser

default_config_file_path = '/data/bb_openpilot.cfg'

class ConfigFile():
  config_file_r = 'r'
  config_file_w = 'w'

  ### Do NOT modify here, modify in /data/bb_openpilot.cfg and reboot
  def read(self, into, config_path):
      configr = configparser.RawConfigParser()
      file_changed = False

      try:
        configr.read(config_path)
        fd = open(config_path, "r")
        prev_file_contents = fd.read()
        fd.close()
      except IOError:
        prev_file_contents = ""
        print("no config file, creating with defaults...")

      main_section = 'OP_CONFIG'
      pref_section = 'OP_PREFERENCES'
      logging_section = 'LOGGING'
      config = configparser.RawConfigParser(allow_no_value=True)
      config.add_section(main_section)
      config.add_section(pref_section)
      config.add_section(logging_section)

      #user_handle -> userHandle
      into.userHandle, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'user_handle', entry_type = str,
        default_value = 'your_tinkla_username',
        comment = 'Username at tinkla.us, for dashboard data and support. If you don\'t have a username, ask for one on Discord, or just enter your Discord handle here.'
      )
      file_changed |= didUpdate

      #force_fingerprint_tesla -> forceFingerprintTesla
      into.forceFingerprintTesla, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'force_fingerprint_tesla', entry_type = bool,
        default_value = False,
        comment = 'Forces the fingerprint to Tesla Model S if OpenPilot fails to identify car via fingerprint.'
      )
      file_changed |= didUpdate

      #force_pedal_over_cc -> forcePedalOverCC
      into.forcePedalOverCC, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'force_pedal_over_cc', entry_type = bool,
        default_value = False,
        comment = 'Forces the use of Tesla Pedal over ACC completely disabling the Tesla CC.'
      )
      file_changed |= didUpdate
      
      #enable_hso -> enableHSO
      into.enableHSO, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'enable_hso', entry_type = bool,
        default_value = True,
        comment = 'Enables Human Steering Override (HSO) feature which allows you to take control of the steering wheel and correct the course of the car without disengaging OpenPilot lane keep assist (LKS, lateral control).'
      )
      file_changed |= didUpdate

      #enable_alca -> enableALCA
      into.enableALCA, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'enable_alca', entry_type = bool,
        default_value = True,
        comment = 'Enables the Adaptive Lane Change Assist (ALCA) feature which will automatically change lanes when driving above 18 MPH (29 km/h) by just pushing 1/2 way on your turn signal stalk; turn signal will remain on for the duration of lane change.'
      )
      file_changed |= didUpdate

      #enable_das_emulation -> enableDasEmulation
      into.enableDasEmulation, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'enable_das_emulation', entry_type = bool,
        default_value = False,
        comment = 'The secret sauce of IC/CID integration; this feature makes the Panda generate all the CAN messages needed for IC/CID integration that mimic the AP interface.'
      )
      file_changed |= didUpdate

      #enable_radar_emulation -> enableRadarEmulation
      into.enableRadarEmulation, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'enable_radar_emulation', entry_type = bool,
        default_value = False,
        comment = 'The secret sauce to make the Tesla Radar work; this feature makes the Panda generate all the CAN messages needed by the Tesla Bosch Radar to operate.'
      )
      file_changed |= didUpdate

      #enable_driver_monitor -> enableDriverMonitor
      into.enableDriverMonitor, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'enable_driver_monitor', entry_type = bool,
        default_value = True,
        comment = 'When turned off, the OpenPilot is tricked into thinking you have your hands on the sterring wheel all the time.'
      )
      file_changed |= didUpdate

      #has_noctua_fan -> hasNoctuaFan
      into.hasNoctuaFan, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'has_noctua_fan', entry_type = bool,
        default_value = False,
        comment = 'Enables control of Noctua fan (at higher RPMS) when you have a Noctua fan installed on an offcial EON fan control module. (Doesn\'t work for FrEONs with Noctua fans.)'
      )
      file_changed |= didUpdate

      #limit_battery_minmax -> limitBatteryMinMax
      into.limitBatteryMinMax, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'limit_battery_minmax', entry_type = bool,
        default_value = True,
        comment = 'Enables battery charging limits; the battery will start charging when battery percentage is below limit_battery_min and will stop charging when battery percentage is above limit_battery_max.'
      )
      file_changed |= didUpdate

      #limit_battery_min -> limitBattery_Min
      into.limitBattery_Min, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'limit_battery_min', entry_type = int,
        default_value = 60,
        comment = 'See limit_battery_minmax.'
      )
      file_changed |= didUpdate

      #limitBattery_Max -> limitBattery_Max
      into.limitBattery_Max, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'limit_battery_max', entry_type = int,
        default_value = 80,
        comment = 'See limit_battery_minmax.'
      )
      file_changed |= didUpdate

      #block_upload_while_tethering -> blockUploadWhileTethering
      into.blockUploadWhileTethering, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'block_upload_while_tethering', entry_type = bool,
        default_value = False,
        comment = 'This setting will block uploading OP videos to Comma when you are tethering through a phone or specified network address. You should set the tether_ip to the first 3 values that your phone provides as IP when you tether. This is phone/carrier specific. For example iPhone addresses start with 172.20.10.x so you would enter 172.20.10.'
      )
      file_changed |= didUpdate

      #tether_ip -> tetherIP
      into.tetherIP, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'tether_ip', entry_type = str,
        default_value = "127.0.0.",
        comment = 'See block_upload_while_tethering.'
      )
      file_changed |= didUpdate

      #use_tesla_gps -> useTeslaGPS
      into.useTeslaGPS, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'use_tesla_gps', entry_type = bool,
        default_value = False,
        comment = 'This setting forces OP to use Tesla GPS data instead of the GPS that comes with the gray panda; both GPS systems use Ublox and both are very close in accuracy; this also allows one to use a White Panda and still have map integration.'
      )
      file_changed |= didUpdate

      #use_tesla_map_data -> useTeslaMapData
      into.useTeslaMapData, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'use_tesla_map_data', entry_type = bool,
        default_value = False,
        comment = 'This setting (which requires root) allows OP to use Tesla navigation map data. (under development)'
      )
      file_changed |= didUpdate

      #has_tesla_ic_integration -> hasTeslaIcIntegration
      into.hasTeslaIcIntegration, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'has_tesla_ic_integration', entry_type = bool,
        default_value = False,
        comment = 'This setting (in conjunction with enable_radar_emulation) helps create the IC integration'
      )
      file_changed |= didUpdate

      #use_tesla_radar -> useTeslaRadar
      into.useTeslaRadar, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'use_tesla_radar', entry_type = bool,
        default_value = False,
        comment = 'Set this setting to True if you have a Tesla Bosch Radar installed (works in conjunction with enable_radar_emulation).'
      )
      file_changed |= didUpdate

      #use_without_harness = useWithoutHarness
      into.useWithoutHarness, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'use_without_harness', entry_type = bool,
        default_value = False,
        comment = 'Not used at the moment; should be False.'
      )
      file_changed |= didUpdate

      #radar_vin -> into.radarVIN
      default_radar_vin = '"                 "'
      into.radarVIN, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'radar_vin', entry_type = str,
        default_value = default_radar_vin,
        comment = 'If you use an aftermarket Tesla Bosch Radar that already has a coded VIN, you will have to enter that VIN value here.'
      )
      file_changed |= didUpdate
      if into.radarVIN == '':
        into.radarVIN = default_radar_vin
        file_changed = True

      #radar_offset -> radarOffset
      into.radarOffset, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'radar_offset', entry_type = float,
        default_value = 0,
        comment = 'If your Tesla Bosch Radar is not centered on the car, this value will allow to enter a correction offset.'
      )
      file_changed |= didUpdate

      #radar_epas_type -> radarEpasType
      into.radarEpasType, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'radar_epas_type', entry_type = int,
        default_value = 0,
        comment = 'Depending on the source of your Tesla Bosch Radar (older or newer Model S or Model X), this setting has to match what the radar was programmed to recognize as EPAS; values are between 0 and 4; finding the right one is trial and error.'
      )
      file_changed |= didUpdate

      #radar_position -> radarPosition
      into.radarPosition, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'radar_position', entry_type = int,
        default_value = 0,
        comment = 'Depending on the source of your Tesla Bosch Radar (older or newer Model S or Model X), this setting has to match what the radar was programmed to have a position (Model S, Model S facelift, Model X); values are between 0 and 3; finding the right one is trial and error.'
      )
      file_changed |= didUpdate

      #fix_1916 -> fix1916
      into.fix1916, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'fix_1916', entry_type = bool,
        default_value = True,
        comment = 'Set this value to False if you are running Tesla software earlier than v2019.16. This fixes the DI_state can message change for DI_cruiseSet which changed in 2019.16 from 9 bits to 8 bits.'
      )
      file_changed |= didUpdate

      #do_auto_update -> doAutoUpdate
      into.doAutoUpdate, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = main_section,
        entry = 'do_auto_update', entry_type = bool,
        default_value = True,
        comment = 'Set this setting to False if you do not want OP to autoupdate every time you reboot and there is a change on the repo.'
      )
      file_changed |= didUpdate
      
      #spiner_text -> spinnerText
      into.spinnerText, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = pref_section,
        entry = 'spinner_text', entry_type = str,
        default_value = '%d',
        comment = 'Here you can customize the text that is shown for the spinner when OP is loading from boot. %d is the current loading percentage.'
      )
      file_changed |= didUpdate

      #hso_numb_period -> hsoNumbPeriod
      into.hsoNumbPeriod, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = pref_section,
        entry = 'hso_numb_period', entry_type = float,
        default_value = 1.5,
        comment = 'Seconds to delay the reengagement of LKAS after turn signal has been used. Time starts when the turn signal is turned on.'
      )
      file_changed |= didUpdate

      #enable_ldw = enableLdw
      into.enableLdw, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = pref_section,
        entry = 'enable_ldw', entry_type = bool,
        default_value = True,
        comment = 'Enable the Lane Departure Warning (LDW) feature; this feature warns the driver if the car gets too close to one of the lane lines when driving above 35 MPH (57 km/h) without touching the steering wheel or when the turn signal is off.'
      )
      file_changed |= didUpdate

      #ldw_numb_period -> ldwNumbPeriod
      into.ldwNumbPeriod, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = pref_section,
        entry = 'ldw_numb_period', entry_type = float,
        default_value = 1.5,
        comment = 'Seconds to disable LDW after the blinker stops blinking.'
      )
      file_changed |= didUpdate

      #tap_blinker_extension -> tapBlinkerExtension
      into.tapBlinkerExtension, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = pref_section,
        entry = 'tap_blinker_extension', entry_type = int,
        default_value = 2,
        comment = 'Number of additional blinks when tapping the turn signal stalk. A value of 2 means 5 blinks total, because the car normally blinks 3 times.'
      )
      file_changed |= didUpdate

      #enable_show_car -> enableShowCar
      into.enableShowCar, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = pref_section,
        entry = 'enable_show_car', entry_type = bool,
        default_value = True,
        comment = 'Shows a Tesla car in OP\'s limited UI mode instead of the triangle that identifies the lead car; this is only used if you do not have IC/CID integration.'
      )
      file_changed |= didUpdate

      #enable_show_logo -> enableShowLogo
      into.enableShowLogo, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = pref_section,
        entry = 'enable_show_logo', entry_type = bool,
        default_value = True,
        comment = 'Shows a Tesla red logo on the EON screen when OP is not engaged.'
      )
      file_changed |= didUpdate

      #ahb_off_duration -> ahbOffDuration
      into.ahbOffDuration, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = pref_section,
        entry = 'ahb_off_duration', entry_type = int,
        default_value = 5,
        comment = 'Duration Auto High Beams (AHB) should be off after last detecting a vehicle. Radar is required to use AHB.'
      )
      file_changed |= didUpdate

      into.shouldLogCanErrors, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = logging_section,
        entry = 'should_log_can_errors', entry_type = bool,
        default_value = False,
        comment = 'Enable CAN error logging to Tinkla'
      )
      file_changed |= didUpdate

      into.shouldLogProcessCommErrors, didUpdate = self.read_config_entry(
        config, configr, prev_file_contents, section = logging_section,
        entry = 'should_log_process_comm_errors', entry_type = bool,
        default_value = False,
        comment = 'Enable Process Comm error logging to Tinkla'
      )
      file_changed |= didUpdate

      if file_changed:
        did_write = True
        with open(config_path, self.config_file_w) as configfile:
          config.write(configfile)
      else:
        did_write = False

      # Remove double quotes from VIN (they are required for empty case)
      into.radarVIN = into.radarVIN.replace('"', '')
      return did_write

  def read_config_entry(self, config, configr, prev_file_contents, section, entry, entry_type, default_value, comment):
      updated = self.update_comment(config, prev_file_contents, section, entry, default_value, comment)
      result = None
      try:
        if entry_type == bool:
          result = configr.getboolean(section, entry)
        elif entry_type == int:
          result = configr.getint(section, entry)
        elif entry_type == float:
          result = configr.getfloat(section, entry)
        else:
          result = configr.get(section, entry)
      except (configparser.NoSectionError, configparser.NoOptionError):
        result = default_value
        updated = True
      config.set(section, entry, result)
      return result, updated

  def update_comment(self, config, prev_file_contents, section, entry, default_value, comment):
      new_comment = ("# " + entry + ": " + comment + " (Default: " + str(default_value) + ")").lower()
      config.set(section, new_comment)
      updated = (prev_file_contents.find(new_comment) == -1)
      return updated

class CarSettings():

  userHandle = None
  forceFingerprintTesla = None
  forcePedalOverCC = None
  enableHSO = None
  enableALCA = None
  enableDasEmulation = None
  enableRadarEmulation = None
  enableDriverMonitor = None
  enableShowCar = None
  enableShowLogo = None
  hasNoctuaFan = None
  limitBatteryMinMax = None
  limitBattery_Min = None
  limitBattery_Max = None
  blockUploadWhileTethering = None
  tetherIP = None
  useTeslaGPS = None
  useTeslaMapData = None
  hasTeslaIcIntegration = None
  useTeslaRadar = None
  useWithoutHarness = None
  radarVIN = None
  enableLdw = None
  radarOffset = None
  radarEpasType = None
  radarPosition = None
  fix1916 = None
  doAutoUpdate = None
  spinnerText = None
  shouldLogProcessCommErrors = None
  shouldLogCanErrors = None
  hsoNumbPeriod = None
  ldwNumbPeriod = None
  tapBlinkerExtension = None
  ahbOffDuration = None

  def __init__(self, optional_config_file_path = default_config_file_path):
    config_file = ConfigFile()
    self.did_write_file = config_file.read(self, config_path = optional_config_file_path)

  def get_value(self, name_of_variable):
    return self.__dict__[name_of_variable]

# Legacy support
def read_config_file(into, config_path = default_config_file_path):
  config_file = ConfigFile()
  config_file.read(into, config_path)
  
