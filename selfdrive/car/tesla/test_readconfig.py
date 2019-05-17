import unittest
import os
import readconfig

class MyTest(unittest.TestCase):
    test_config_file = "./test_config_file.cfg"

    def setUp(self):
        self.delete_test_config_file()
    
    def tearDown(self):
        self.delete_test_config_file()

    # Tests to make sure defaults are set when the config file is missing
    def test_defaults_missing_file(self):
        cs = readconfig.CarSettings(optional_config_file_path = self.test_config_file)
        self.check_defaults(cs)

    # Tests to make sure defaults are set when the config file is not missing
    def test_defaults_empty_file(self):
        config_file_path = "./test_config_file2.cfg"
        self.create_empty_config_file(config_file_path)
        cs = readconfig.CarSettings(optional_config_file_path = config_file_path)
        self.check_defaults(cs)
        os.remove(config_file_path)

    # Tests to make sure existing parameters in config file are not overriden when loading defaults
    def test_defaults_non_overriding(self):
        config_file_path = "./test_config_file2.cfg"
        self.create_empty_config_file(config_file_path, test_parameter_string = "force_pedal_over_cc = True")
        cs = readconfig.CarSettings(optional_config_file_path = config_file_path)
        # Should still be true, even though the defaut is False
        self.assertEqual(cs.forcePedalOverCC, True)
        os.remove(config_file_path)

    # Tests to make sure empty (default) radar vin is correctly stored and read back
    def test_empty_radar_vin(self):
        self.delete_test_config_file()
        # first pass creates config file
        cs = readconfig.CarSettings(optional_config_file_path = self.test_config_file)
        self.assertEqual(cs.radarVIN, "                 ")
        # second pass actually reads the file
        cs = readconfig.CarSettings(optional_config_file_path = self.test_config_file)
        self.assertEqual(cs.radarVIN, "                 ")

    # Test to make sure old radarVIN entries are converted to the new format (with double quotes for empty string)
    def test_update_empty_radar_vin(self):
        config_file_path = "./test_config_file2.cfg"
        self.create_empty_config_file(config_file_path, test_parameter_string = "radar_vin =                 ")
        cs = readconfig.CarSettings(optional_config_file_path = config_file_path)
        # Should be the correct all spaces VIN
        self.assertEqual(cs.radarVIN, "                 ")
        os.remove(config_file_path)

    # Test to make sure radarVIN is read correctly
    def test_radar_vin_with_data(self):
        config_file_path = "./test_config_file2.cfg"
        self.create_empty_config_file(config_file_path, test_parameter_string = "radar_vin = 12345678901234567")
        cs = readconfig.CarSettings(optional_config_file_path = config_file_path)
        self.assertEqual(cs.radarVIN, "12345678901234567")
        os.remove(config_file_path)

    #def test_readconfig_no_arguments(self):
    #    cs1 = readconfig.CarSettings()
    #    cs2 = read_config_file()

    def check_defaults(self, cs):
        self.assertEqual(cs.forcePedalOverCC, False)
        self.assertEqual(cs.enableHSO, True)
        self.assertEqual(cs.enableALCA, True)
        self.assertEqual(cs.enableDasEmulation, False)
        self.assertEqual(cs.enableRadarEmulation, False)
        self.assertEqual(cs.enableSpeedVariableDesAngle, True)
        self.assertEqual(cs.enableRollAngleCorrection, False)
        self.assertEqual(cs.enableFeedForwardAngleCorrection, True)
        self.assertEqual(cs.enableDriverMonitor, True)
        self.assertEqual(cs.enableShowCar, True)
        self.assertEqual(cs.enableShowLogo, True)
        self.assertEqual(cs.hasNoctuaFan, False)
        self.assertEqual(cs.limitBatteryMinMax, True)
        self.assertEqual(cs.limitBattery_Min, 60)
        self.assertEqual(cs.limitBattery_Max, 80)
        self.assertEqual(cs.blockUploadWhileTethering, False)
        self.assertEqual(cs.tetherIP, "127.0.0.")
        self.assertEqual(cs.useTeslaGPS, False)
        self.assertEqual(cs.useTeslaMapData, False)
        self.assertEqual(cs.hasTeslaIcIntegration, False)
        self.assertEqual(cs.useAnalogWhenNoEon, False)
        self.assertEqual(cs.useTeslaRadar, False)
        self.assertEqual(cs.useWithoutHarness, False)
        self.assertEqual(cs.radarVIN, "                 ")
        self.assertEqual(cs.enableLdw, True)
        self.assertEqual(cs.radarOffset, 0)
        self.assertEqual(cs.radarEpasType, 0)
        self.assertEqual(cs.radarPosition, 0)
        self.assertEqual(cs.doAutoUpdate, True)

    # Helper methods

    def delete_test_config_file(self):
        if os.path.exists(self.test_config_file):
            os.remove(self.test_config_file)

    def create_empty_config_file(self, file_name, test_parameter_string = ""):
        if os.path.exists(file_name):
            os.remove(file_name)
        file = open(file_name, "a")
        file.write("[OP_CONFIG]\n")
        file.write(test_parameter_string + "\n")
        file.close() 
