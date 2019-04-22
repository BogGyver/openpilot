import ConfigParser

config_path = '/data/bb_openpilot.cfg'
config_file_r = 'r'
config_file_w = 'wb'

def read_config_file(CS):
    file_changed = False
    configr = ConfigParser.ConfigParser()
    try:
      configr.read(config_path)
    except:
      file_changed = True
      print "no config file, creating with defaults..."
    config = ConfigParser.RawConfigParser()
    config.add_section('OP_CONFIG')
    
    #force_pedal_over_cc -> CS.forcePedalOverCC
    try:
      CS.forcePedalOverCC = configr.getboolean('OP_CONFIG','force_pedal_over_cc')
    except:
      CS.forcePedalOverCC = True
      file_changed = True
    config.set('OP_CONFIG', 'force_pedal_over_cc', CS.forcePedalOverCC)
    
    #enable_hso -> CS.enableHSO
    try:
      CS.enableHSO = configr.getboolean('OP_CONFIG','enable_hso')
    except:
      CS.enableHSO = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_hso', CS.enableHSO)

    #enable_alca -> CS.enableALCA
    try:
      CS.enableALCA = configr.getboolean('OP_CONFIG','enable_alca')
    except:
      CS.enableALCA = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_alca', CS.enableALCA)

    #enable_das_emulation -> CS.enableDasEmulation
    try:
      CS.enableDasEmulation = configr.getboolean('OP_CONFIG','enable_das_emulation')
    except:
      CS.enableDasEmulation = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_das_emulation', CS.enableDasEmulation)

    #enable_radar_emulation -> CS.enableRadarEmulation
    try:
      CS.enableRadarEmulation = configr.getboolean('OP_CONFIG','enable_radar_emulation')
    except:
      CS.enableRadarEmulation = True
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

    #enable_driver_monitor -> CS.enableDriverMonitor
    try:
      CS.enableDriverMonitor = configr.getboolean('OP_CONFIG','enable_driver_monitor')
    except:
      CS.enableDriverMonitor = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_driver_monitor', CS.enableDriverMonitor)

    #enable_show_car -> CS.enableShowCar
    try:
      CS.enableShowCar = configr.getboolean('OP_CONFIG','enable_show_car')
    except:
      CS.enableShowCar = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_show_car', CS.enableShowCar)

    #enable_show_logo -> CS.enableShowLogo
    try:
      CS.enableShowLogo = configr.getboolean('OP_CONFIG','enable_show_logo')
    except:
      CS.enableShowLogo = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_show_logo', CS.enableShowLogo)

    #has_noctua_fan -> CS.hasNoctuaFan
    try:
      CS.hasNoctuaFan = configr.getboolean('OP_CONFIG','has_noctua_fan')
    except:
      CS.hasNoctuaFan = False
      file_changed = True
    config.set('OP_CONFIG', 'has_noctua_fan', CS.hasNoctuaFan)

    #limit_battery_minmax -> CS.limitBatteryMinMax
    try:
      CS.limitBatteryMinMax = configr.getboolean('OP_CONFIG','limit_battery_minmax')
    except:
      CS.limitBatteryMinMax = False
      file_changed = True
    config.set('OP_CONFIG', 'limit_battery_minmax', CS.limitBatteryMinMax)

    #limit_battery_min -> CS.limitBattery_Min
    try:
      CS.limitBattery_Min = configr.getint('OP_CONFIG','limit_battery_min')
    except:
      CS.limitBattery_Min = 60
      file_changed = True
    config.set('OP_CONFIG', 'limit_battery_min', CS.limitBattery_Min)

    #limit_battery_max -> CS.limitBattery_Max
    try:
      CS.limitBattery_Max = configr.getint('OP_CONFIG','limit_battery_max')
    except:
      CS.limitBattery_Max = 70
      file_changed = True
    config.set('OP_CONFIG', 'limit_battery_max', CS.limitBattery_Max)

    #block_upload_while_tethering -> CS.blockUploadWhileTethering
    try:
      CS.blockUploadWhileTethering = configr.getboolean('OP_CONFIG','block_upload_while_tethering')
    except:
      CS.blockUploadWhileTethering = False
      file_changed = True
    config.set('OP_CONFIG', 'block_upload_while_tethering', CS.blockUploadWhileTethering)

    #tether_ip -> CS.tetherIP
    try:
      CS.tetherIP = configr.get('OP_CONFIG','tether_ip')
    except:
      CS.tetherIP = "127.0.0."
      file_changed = True
    config.set('OP_CONFIG', 'tether_ip', CS.tetherIP)

    #use_tesla_gps -> CS.useTeslaGPS
    try:
      CS.useTeslaGPS = configr.getboolean('OP_CONFIG','use_tesla_gps')
    except:
      CS.useTeslaGPS = False
      file_changed = True
    config.set('OP_CONFIG', 'use_tesla_gps', CS.useTeslaGPS)

    #use_tesla_map_data -> CS.useTeslaMapData
    try:
      CS.useTeslaMapData = configr.getboolean('OP_CONFIG','use_tesla_map_data')
    except:
      CS.useTeslaMapData = False
      file_changed = True
    config.set('OP_CONFIG', 'use_tesla_map_data', CS.useTeslaMapData)

    #has_tesla_IC_integration -> CS.hasTeslaIcIntegration
    try:
      CS.hasTeslaIcIntegration = configr.getboolean('OP_CONFIG','has_tesla_ic_integration')
    except:
      CS.hasTeslaIcIntegration = False
      file_changed = True
    config.set('OP_CONFIG', 'has_tesla_ic_integration', CS.hasTeslaIcIntegration)

    #use_analog_when_no_eon -> CS.useAnalogWhenNoEon
    try:
      CS.useAnalogWhenNoEon = configr.getboolean('OP_CONFIG','use_analog_when_no_eon')
    except:
      CS.useAnalogWhenNoEon = False
      file_changed = True
    config.set('OP_CONFIG', 'use_analog_when_no_eon', CS.useAnalogWhenNoEon)
    
    #use_tesla_radar -> CS.useTeslaRadar
    try:
      CS.useTeslaRadar = configr.getboolean('OP_CONFIG','use_tesla_radar')
    except:
      CS.useTeslaRadar = False
      file_changed = True
    config.set('OP_CONFIG', 'use_tesla_radar', CS.useTeslaRadar)

    #use_without_harness = CS.useWithoutHarness
    try:
      CS.useWithoutHarness = configr.getboolean('OP_CONFIG','use_without_harness')
    except:
      CS.useWithoutHarness = False
      file_changed = True
    config.set('OP_CONFIG', 'use_without_harness', CS.useWithoutHarness)

    #radar_vin -> CS.radarVIN
    try:
      CS.radarVIN = configr.get('OP_CONFIG','radar_vin')
    except:
      CS.radarVIN = "                 "
      file_changed = True
    config.set('OP_CONFIG', 'radar_vin', CS.radarVIN)

    #enable_ldw = CS.enableLdw
    try:
      CS.enableLdw = configr.getboolean('OP_CONFIG','enable_ldw')
    except:
      CS.enableLdw = True
      file_changed = True
    config.set('OP_CONFIG', 'enable_ldw', CS.enableLdw)

    #radar_offset -> CS.radarOffset
    try:
      CS.radarOffset = configr.getint('OP_CONFIG','radar_offset')
    except:
      CS.radarOffset = 0.
      file_changed = True
    config.set('OP_CONFIG', 'radar_offset', CS.radarOffset)

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

class CarSettings(object):
  def __init__(self):
    ### START OF MAIN CONFIG OPTIONS ###
    ### Do NOT modify here, modify in /data/bb_openpilot.cfg and reboot
    self.forcePedalOverCC = True
    self.enableHSO = True 
    self.enableALCA = True
    self.enableDasEmulation = True
    self.enableRadarEmulation = True
    self.enableSpeedVariableDesAngle = False
    self.enableRollAngleCorrection = False
    self.enableFeedForwardAngleCorrection = True
    self.enableDriverMonitor = True
    self.enableShowCar = True
    self.enableShowLogo = True
    self.hasNoctuaFan = False
    self.limitBatteryMinMax = False
    self.limitBattery_Min = 60
    self.limitBattery_Max = 70
    self.doAutoUpdate = True
    self.blockUploadWhileTethering = False
    self.tetherIP = "127.0.0."
    self.useTeslaGPS = False
    self.useTeslaMapData = False
    self.hasTeslaIcIntegration = False
    self.useAnalogWhenNoEon = False
    self.useTeslaRadar = False
    self.useWithoutHarness = False
    self.radarVIN = "                 "
    self.enableLdw = True
    self.radarOffset = 0
    #read config file
    read_config_file(self)
    ### END OF MAIN CONFIG OPTIONS ###

  def get_value(self,name_of_variable):
    return_val = None
    exec("%s = self.%s" % ('return_val',name_of_variable))
    return return_val
    
