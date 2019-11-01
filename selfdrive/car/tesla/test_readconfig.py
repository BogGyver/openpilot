#!/usr/bin/env python3.7

import unittest
import os
import readconfig
from selfdrive.car.tesla.readconfig import read_config_file, CarSettings


class CarSettingsTestClass():
  forcePedalOverCC = None
  fix1916 = None

  def __init__(self):
    pass


class ReadConfigTests(unittest.TestCase):
  test_config_file = "./test_config_file.cfg"

  def setUp(self):
    self.delete_test_config_file()

  def tearDown(self):
    self.delete_test_config_file()

  # Tests to make sure defaults are set when the config file is missing
  def test_defaults_missing_file(self):
    # First time proves that data is set locally
    cs = readconfig.CarSettings(
        optional_config_file_path=self.test_config_file)
    self.check_defaults(cs)
    self.assertEqual(cs.did_write_file, True)
    # Run a second time to make sure it was saved and read correctly
    cs = readconfig.CarSettings(
        optional_config_file_path=self.test_config_file)
    self.check_defaults(cs)
    self.assertEqual(cs.did_write_file, False)

  # Tests to make sure defaults are set when the config file is not missing
  def test_defaults_empty_file(self):
    config_file_path = "./test_config_file2.cfg"
    self.create_empty_config_file(config_file_path)
    cs = readconfig.CarSettings(optional_config_file_path=config_file_path)
    self.check_defaults(cs)
    os.remove(config_file_path)

  # Tests to make sure existing parameters in config file are not overriden when loading defaults
  def test_defaults_non_overriding(self):
    config_file_path = "./test_config_file2.cfg"
    self.create_empty_config_file(
        config_file_path, test_parameter_string="force_pedal_over_cc = True")
    cs = readconfig.CarSettings(optional_config_file_path=config_file_path)
    # Should still be true, even though the defaut is False
    self.assertEqual(cs.forcePedalOverCC, True)
    os.remove(config_file_path)

  # Tests to make sure empty (default) radar vin is correctly stored and read back
  def test_empty_radar_vin(self):
    self.delete_test_config_file()
    # first pass creates config file
    cs = readconfig.CarSettings(
        optional_config_file_path=self.test_config_file)
    self.assertEqual(cs.radarVIN, "                 ")
    # second pass actually reads the file
    cs = readconfig.CarSettings(
        optional_config_file_path=self.test_config_file)
    self.assertEqual(cs.radarVIN, "                 ")

  # Test to make sure old radarVIN entries are converted to the new format (with double quotes for empty string)
  def test_update_empty_radar_vin(self):
    config_file_path = "./test_config_file2.cfg"
    self.create_empty_config_file(
        config_file_path, test_parameter_string="radar_vin =                 ")
    cs = readconfig.CarSettings(optional_config_file_path=config_file_path)
    # Should be the correct all spaces VIN
    self.assertEqual(cs.radarVIN, "                 ")
    os.remove(config_file_path)

  # Test to make sure radarVIN is read correctly
  def test_radar_vin_with_data(self):
    config_file_path = "./test_config_file2.cfg"
    self.create_empty_config_file(
        config_file_path,
        test_parameter_string="radar_vin = 12345678901234567")
    cs = readconfig.CarSettings(optional_config_file_path=config_file_path)
    self.assertEqual(cs.radarVIN, "12345678901234567")
    os.remove(config_file_path)

  def test_comments(self):
    expected_comment = "# do_auto_update: set this setting to false if you do not want op to autoupdate every time you reboot and there is a change on the repo (default: true)"
    config_file_path = "./test_config_file2.cfg"
    self.create_empty_config_file(
        config_file_path, test_parameter_string="force_pedal_over_cc = True")
    cs = readconfig.CarSettings(optional_config_file_path=config_file_path)
    # Should still be true, even though the defaut is False
    self.assertEqual(cs.forcePedalOverCC, True)
    # Make sure comment was added:
    fd = open(config_file_path, "r")
    contents = fd.read()
    fd.close()
    self.assertNotEqual(contents.find(expected_comment), -1)
    os.remove(config_file_path)

  def test_comment_update(self):
    config_file_path = "./test_config_file3.cfg"
    old_comment = "# do_auto_update - old description (default: true)"
    old_entry = "do_auto_update = False"
    self.create_empty_config_file(config_file_path,
                                  test_parameter_string="[OP_CONFIG]\n" +
                                  old_comment + "\n" + old_entry)
    expected_comment = "# do_auto_update: set this setting to false if you do not want op to autoupdate every time you reboot and there is a change on the repo (default: true)"
    cs = CarSettings(optional_config_file_path=config_file_path)
    # Make sure setting didn't change
    self.assertEqual(cs.doAutoUpdate, False)
    # Make sure comment was updated:
    fd = open(config_file_path, "r")
    contents = fd.read()
    fd.close()
    self.assertTrue(contents.find(expected_comment) != -1)
    # File should have changed (to update comment)
    self.assertEqual(cs.did_write_file, True)
    # Next time we read, file shouldn't change anymore:
    cs = readconfig.CarSettings(optional_config_file_path=config_file_path)
    self.assertEqual(cs.did_write_file, False)
    # Now remove a config option to cause an update:
    fd = open(config_file_path, "r")
    contents = fd.read()
    fd.close()
    new_contents = contents.replace("limit_battery_max = 80", "")
    os.remove(config_file_path)
    fd = open(config_file_path, "wb")
    fd.write(new_contents)
    fd.close()
    cs = readconfig.CarSettings(optional_config_file_path=config_file_path)
    another_comment = '# radar_vin: if you used an aftermarket tesla bosch radar that already has a coded vin, you will have to enter that vin value here (default: "                 ")'
    # Make sure other comments were written:
    fd = open(config_file_path, "r")
    contents = fd.read()
    fd.close()
    self.assertTrue(contents.find(another_comment) != -1)
    os.remove(config_file_path)

  def test_float_parsing(self):
    config_file_path = "./test_config_file2.cfg"
    self.create_empty_config_file(config_file_path,
                                  test_parameter_string="radar_offset = 3.14")
    cs = readconfig.CarSettings(optional_config_file_path=config_file_path)
    self.assertEqual(cs.radarOffset, 3.14)
    os.remove(config_file_path)

  # Make sure existing calls to CarSettings. read_config_file
  # continue to work with no changes
  def test_readconfig_no_arguments(self):
    config_file_path = "./test_config_file2.cfg"
    self.create_empty_config_file(
        config_file_path, test_parameter_string="force_pedal_over_cc = True")
    #cs = readconfig.CarSettings(optional_config_file_path = config_file_path)
    cs = CarSettingsTestClass()
    try:
      read_config_file(cs)
    except IOError:
      pass    # IOError is expected if running pytest outside of EON environment
    # Make sure calling read_config_files directly works as expected
    read_config_file(cs, config_path=config_file_path)
    self.assertEqual(cs.forcePedalOverCC, True)
    read_config_file(cs, config_path=self.test_config_file)
    self.assertEqual(cs.forcePedalOverCC, False)
    # check for defaults
    self.assertEqual(cs.fix1916, False)
    os.remove(config_file_path)

  # Test get_value interface:
  def test_get_value(self):
    config_file_path = "./test_config_file3.cfg"
    cs = readconfig.CarSettings(optional_config_file_path=config_file_path)
    value = cs.get_value("userHandle")
    self.assertEqual(value, 'your_tinkla_username')
    value = cs.get_value("doAutoUpdate")
    self.assertEqual(value, True)
    os.remove(config_file_path)

  def check_defaults(self, cs):
    self.assertEqual(cs.userHandle, 'your_tinkla_username')
    self.assertEqual(cs.forceFingerprintTesla, False)
    self.assertEqual(cs.forcePedalOverCC, False)
    self.assertEqual(cs.enableHSO, True)
    self.assertEqual(cs.enableALCA, True)
    self.assertEqual(cs.enableDasEmulation, False)
    self.assertEqual(cs.enableRadarEmulation, False)
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
    self.assertEqual(cs.useTeslaRadar, False)
    self.assertEqual(cs.useWithoutHarness, False)
    self.assertEqual(cs.radarVIN, "                 ")
    self.assertEqual(cs.enableLdw, True)
    self.assertEqual(cs.radarOffset, 0)
    self.assertEqual(cs.radarEpasType, 0)
    self.assertEqual(cs.radarPosition, 0)
    self.assertEqual(cs.doAutoUpdate, True)
    self.assertEqual(cs.fix1916, False)
    self.assertEqual(cs.get_value("userHandle"), 'your_tinkla_username')
    self.assertEqual(cs.get_value("doAutoUpdate"), True)
    self.assertEqual(cs.shouldLogCanErrors, False)
    self.assertEqual(cs.shouldLogProcessCommErrors, False)

  # Helper methods

  def delete_test_config_file(self):
    if os.path.exists(self.test_config_file):
      os.remove(self.test_config_file)

  def create_empty_config_file(self, file_name, test_parameter_string=""):
    if os.path.exists(file_name):
      os.remove(file_name)
    fp = open(file_name, "a")
    fp.write("[OP_CONFIG]\n")
    fp.write(test_parameter_string + "\n")
    fp.close()


# Allow running the tests directly via unittests when pytest is not available
if __name__ == '__main__':
  unittest.main()
