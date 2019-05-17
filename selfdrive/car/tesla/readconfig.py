import ConfigParser

default_config_file_path = '/data/bb_openpilot.cfg'
config_file_r = 'r'
config_file_w = 'wb'

### Do NOT modify here, modify in /data/bb_openpilot.cfg and reboot
def read_config_file(CS, config_path):
    file_changed = False
    configr = ConfigParser.ConfigParser()

    try:
      configr.read(config_path)
    except:
      file_changed = True
      print "no config file, creating with defaults..."
    config = ConfigParser.RawConfigParser()
    config.add_section('OP_CONFIG')
    
    #force_pedal_over_cc - Forces the use of Tesla Pedal over ACC completely disabling the Tesla CC.
    #force_pedal_over_cc -> CS.forcePedalOverCC
    try:
      CS.forcePedalOverCC = configr.getboolean('OP_CONFIG','force_pedal_over_cc')
    except:
      CS.forcePedalOverCC = False
      file_changed = True
    config.set('OP_CONFIG', 'force_pedal_over_cc', CS.forcePedalOverCC)
    
    #enable_hso - Enables Human Steering Override (HSO) feature which allows you to take control of the steering wheel and correct the course of the car without disengaging OpenPilot lane keep assis (LKS, lateral control)
    #enable_hso -> CS.enableHSO
    try:
      CS.enableHSO = configr.getboolean('OP_CONFIG','enable_hso')
    except:
      CS.enableHSO = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_hso', CS.enableHSO)

    #enable_alca - Enables the Adaptive Lane Change Assist (ALCA) feature which will automatically change lanes when driving above 18 MPH (29 km/h) by just pushing 1/2 way on your turn signal stalk; turn signal will remain on for the duration of lane change
    #enable_alca -> CS.enableALCA
    try:
      CS.enableALCA = configr.getboolean('OP_CONFIG','enable_alca')
    except:
      CS.enableALCA = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_alca', CS.enableALCA)

    #enable_das_emulation - The secret sauce of IC/CID integration; this feature makes the Panda generate all the CAN messages needed for IC/CID integration that mimics the AP interface
    #enable_das_emulation -> CS.enableDasEmulation
    try:
      CS.enableDasEmulation = configr.getboolean('OP_CONFIG','enable_das_emulation')
    except:
      CS.enableDasEmulation = False
      file_changed = True
    config.set('OP_CONFIG', 'enable_das_emulation', CS.enableDasEmulation)

    #enable_radar_emulation - The secret sauce to make the Tesla Radar work; this feature make the Panda generate all the CAN messages needed by the Tesla Bosch Radar to operate
    #enable_radar_emulation -> CS.enableRadarEmulation
    try:
      CS.enableRadarEmulation = configr.getboolean('OP_CONFIG','enable_radar_emulation')
    except:
      CS.enableRadarEmulation = False
      file_changed = True
    config.set('OP_CONFIG', 'enable_radar_emulation', CS.enableRadarEmulation)

    #enable_speed_variable_angle -> CS.enableSpeedVariableDesAngle
    try:
      CS.enableSpeedVariableDesAngle = configr.getboolean('OP_CONFIG','enable_speed_variable_angle')
    except:
      CS.enableSpeedVariableDesAngle = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_speed_variable_angle', CS.enableSpeedVariableDesAngle)

    #enable_roll_angle_correction -> CS.enableRollAngleCorrection
    try:
      CS.enableRollAngleCorrection = configr.getboolean('OP_CONFIG','enable_roll_angle_correction')
    except:
      CS.enableRollAngleCorrection = False
      file_changed = True
    config.set('OP_CONFIG', 'enable_roll_angle_correction', CS.enableRollAngleCorrection)

    #enable_feed_forward_angle_correction -> CS.enableFeedForwardAngleCorrection
    try:
      CS.enableFeedForwardAngleCorrection = configr.getboolean('OP_CONFIG','enable_feed_forward_angle_correction')
    except:
      CS.enableFeedForwardAngleCorrection = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_feed_forward_angle_correction', CS.enableFeedForwardAngleCorrection)

    #enable_driver_monitor - When turned off, the OpenPilot is tricked into thinking you have the hands on the sterring wheel all the time
    #enable_driver_monitor -> CS.enableDriverMonitor
    try:
      CS.enableDriverMonitor = configr.getboolean('OP_CONFIG','enable_driver_monitor')
    except:
      CS.enableDriverMonitor = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_driver_monitor', CS.enableDriverMonitor)

    #enable_show_car - Shows a Tesla car in the limitted UI mode instead of the triangle that identifies the lead car; this is only used if you do not have IC/CID integration
    #enable_show_car -> CS.enableShowCar
    try:
      CS.enableShowCar = configr.getboolean('OP_CONFIG','enable_show_car')
    except:
      CS.enableShowCar = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_show_car', CS.enableShowCar)

    #enable_show_logo - Shows a Tesla red logo on the EON screen when OP is not enabled
    #enable_show_logo -> CS.enableShowLogo
    try:
      CS.enableShowLogo = configr.getboolean('OP_CONFIG','enable_show_logo')
    except:
      CS.enableShowLogo = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_show_logo', CS.enableShowLogo)

    #has_noctua_fan - Enables control of Noctua fan (at higher RPMS) when you have a Noctua fan installed
    #has_noctua_fan -> CS.hasNoctuaFan
    try:
      CS.hasNoctuaFan = configr.getboolean('OP_CONFIG','has_noctua_fan')
    except:
      CS.hasNoctuaFan = False
      file_changed = True
    config.set('OP_CONFIG', 'has_noctua_fan', CS.hasNoctuaFan)

    #limit_battery_minmax - Enables battery charging limits; the battery will start charging when battery % is below limit_battery_min and will stop charging when battery % is above limit_battery_max
    #limit_battery_minmax -> CS.limitBatteryMinMax
    try:
      CS.limitBatteryMinMax = configr.getboolean('OP_CONFIG','limit_battery_minmax')
    except:
      CS.limitBatteryMinMax = True
      file_changed = True
    config.set('OP_CONFIG', 'limit_battery_minmax', CS.limitBatteryMinMax)

    #limit_battery_min - See limit_battery_minmax
    #limit_battery_min -> CS.limitBattery_Min
    try:
      CS.limitBattery_Min = configr.getint('OP_CONFIG','limit_battery_min')
    except:
      CS.limitBattery_Min = 60
      file_changed = True
    config.set('OP_CONFIG', 'limit_battery_min', CS.limitBattery_Min)

    #limit_battery_max - See limit_battery_minmax
    #limit_battery_max -> CS.limitBattery_Max
    try:
      CS.limitBattery_Max = configr.getint('OP_CONFIG','limit_battery_max')
    except:
      CS.limitBattery_Max = 80
      file_changed = True
    config.set('OP_CONFIG', 'limit_battery_max', CS.limitBattery_Max)

    #block_upload_while_tethering - This setting will block uploading OP videos to Comma when you are tethering through the phone. You should set the tether_ip to the first 3 values that your phone provides as IP when you tether. This is phone/carrier specific. For example iPhone give addresses like 172.20.10.x so you would enter 172.20.10.
    #block_upload_while_tethering -> CS.blockUploadWhileTethering
    try:
      CS.blockUploadWhileTethering = configr.getboolean('OP_CONFIG','block_upload_while_tethering')
    except:
      CS.blockUploadWhileTethering = False
      file_changed = True
    config.set('OP_CONFIG', 'block_upload_while_tethering', CS.blockUploadWhileTethering)

    #tether_ip - See block_upload_while_tethering
    #tether_ip -> CS.tetherIP
    try:
      CS.tetherIP = configr.get('OP_CONFIG','tether_ip')
    except:
      CS.tetherIP = "127.0.0."
      file_changed = True
    config.set('OP_CONFIG', 'tether_ip', CS.tetherIP)

    #use_tesla_gps - This setting makes OP to use Tesla GPS data instead of the GPS that comes with the gray panda; both GPS systems use Ublox and both are very close in accuracy; this also allows one to use a White Panda and still have map integration
    #use_tesla_gps -> CS.useTeslaGPS
    try:
      CS.useTeslaGPS = configr.getboolean('OP_CONFIG','use_tesla_gps')
    except:
      CS.useTeslaGPS = False
      file_changed = True
    config.set('OP_CONFIG', 'use_tesla_gps', CS.useTeslaGPS)

    #use_tesla_map_data - This setting (which requires root) allows OP to use Tesla navigation map data (under development)
    #use_tesla_map_data -> CS.useTeslaMapData
    try:
      CS.useTeslaMapData = configr.getboolean('OP_CONFIG','use_tesla_map_data')
    except:
      CS.useTeslaMapData = False
      file_changed = True
    config.set('OP_CONFIG', 'use_tesla_map_data', CS.useTeslaMapData)

    #has_tesla_ic_integration - This setting (in conjunction with enable_radar_emulation) help create the IC integration
    #has_tesla_IC_integration -> CS.hasTeslaIcIntegration
    try:
      CS.hasTeslaIcIntegration = configr.getboolean('OP_CONFIG','has_tesla_ic_integration')
    except:
      CS.hasTeslaIcIntegration = False
      file_changed = True
    config.set('OP_CONFIG', 'has_tesla_ic_integration', CS.hasTeslaIcIntegration)

    #use_analog_when_no_eon - Not used at the moment; should be False
    #use_analog_when_no_eon -> CS.useAnalogWhenNoEon
    try:
      CS.useAnalogWhenNoEon = configr.getboolean('OP_CONFIG','use_analog_when_no_eon')
    except:
      CS.useAnalogWhenNoEon = False
      file_changed = True
    config.set('OP_CONFIG', 'use_analog_when_no_eon', CS.useAnalogWhenNoEon)
    
    #use_tesla_radar - Set this setting to True if you have a Tesla Bosch Radar installed (works in conjunction with enable_radar_emulation)
    #use_tesla_radar -> CS.useTeslaRadar
    try:
      CS.useTeslaRadar = configr.getboolean('OP_CONFIG','use_tesla_radar')
    except:
      CS.useTeslaRadar = False
      file_changed = True
    config.set('OP_CONFIG', 'use_tesla_radar', CS.useTeslaRadar)

    #use_without_harness - Not used at the moment; should be False
    #use_without_harness = CS.useWithoutHarness
    try:
      CS.useWithoutHarness = configr.getboolean('OP_CONFIG','use_without_harness')
    except:
      CS.useWithoutHarness = False
      file_changed = True
    config.set('OP_CONFIG', 'use_without_harness', CS.useWithoutHarness)

    #radar_vin - If you used an aftermarket Tesla Bosch Radar that already has a coded VIN, you will have to enter that VIN value here
    #radar_vin -> CS.radarVIN
    try:
      CS.radarVIN = configr.get('OP_CONFIG','radar_vin')
      if CS.radarVIN == '':
        CS.radarVIN = '"                 "'
        file_changed = True
    except:
      CS.radarVIN = '"                 "'
      file_changed = True
    config.set('OP_CONFIG', 'radar_vin', CS.radarVIN)

    #enable_ldw - Enable the Lane Departure Warning (LDW) feature; this feature warns the driver is the car gets too close to one of the lines when driving above 45 MPH (72 km/h) without touching the steering wheel and when the turn signal is off
    #enable_ldw = CS.enableLdw
    try:
      CS.enableLdw = configr.getboolean('OP_CONFIG','enable_ldw')
    except:
      CS.enableLdw = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_ldw', CS.enableLdw)

    #radar_offset - If your Tesla Bosch Radar is not centered on the car, this value will allow to enter a correction offset
    #radar_offset -> CS.radarOffset
    try:
      CS.radarOffset = configr.getfloat('OP_CONFIG','radar_offset')
    except:
      CS.radarOffset = 0.
      file_changed = True
    config.set('OP_CONFIG', 'radar_offset', CS.radarOffset)

    #radar_epas_type - Depending on the source of your Tesla Bosch Radar (older or newer Model S or Model X), this setting has to match what the radar was programmed to recognize as EPAS; values are between 0 and 4; finding the right one is trial and error
    #radar_epas_type -> CS.radarEpasType
    try:
      CS.radarEpasType = configr.getint('OP_CONFIG','radar_epas_type')
    except:
      CS.radarEpasType = 0
      file_changed = True
    config.set('OP_CONFIG', 'radar_epas_type', CS.radarEpasType)
    
    #radar_position - Depending on the source of your Tesla Bosch Radar (older or newer Model S or Model X), this setting has to match what the radar was programmed to have a position (Model S, Model S facelift, Model X); values are between 0 and 3; finding the right one is trial and error
    #radar_position -> CS.radarPosition
    try:
      CS.radarPosition = configr.getint('OP_CONFIG','radar_position')
    except:
      CS.radarPosition = 0
      file_changed = True
    config.set('OP_CONFIG', 'radar_position', CS.radarPosition)

    #do_auto_update - Set this setting to False if you do not want OP to autoupdate every time you reboot and there is a change on the repo
    #do_auto_update -> CS.doAutoUpdate
    try:
      CS.doAutoUpdate = configr.getboolean('OP_CONFIG','do_auto_update')
    except:
      CS.doAutoUpdate = True
      file_changed = True
    config.set('OP_CONFIG', 'do_auto_update', CS.doAutoUpdate)

    if file_changed:
      with open(config_path, config_file_w) as configfile:
        config.write(configfile)

    # Update for radar VIN
    CS.radarVIN = CS.radarVIN.replace('"', '')

class CarSettings(object):
  def __init__(self, optional_config_file_path = default_config_file_path):
    read_config_file(self, config_path = optional_config_file_path)

  def get_value(self,name_of_variable):
    return_val = None
    exec("%s = self.%s" % ('return_val',name_of_variable))
    return return_val
    
