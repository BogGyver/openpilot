import configparser
import subprocess
from common.basedir import BASEDIR

default_config_file_path = "%s/../openpilot.cfg" % BASEDIR


class ConfigFile:
    config_file_r = "r"
    config_file_w = "w"

    ### Do NOT modify here, modify in /data/openpilot.cfg and reboot
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

        main_section = "OP_CONFIG"
        pref_section = "OP_PREFERENCES"
        #jetson_section = "JETSON_PREFERENCES"
        #logging_section = "LOGGING"
        config = configparser.RawConfigParser(allow_no_value=True)
        config.add_section(main_section)
        config.add_section(pref_section)
        #config.add_section(jetson_section)
        #config.add_section(logging_section)

        # uses_a_pillar_harness -> usesApillarHarness
        into.usesApillarHarness, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="uses_a_pillar_harness",
            entry_type=bool,
            default_value=False,
            comment="Enable when using the new A pillar harness.",
        )
        file_changed |= didUpdate

        # force_pedal_over_cc -> forcePedalOverCC
        into.forcePedalOverCC, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="force_pedal_over_cc",
            entry_type=bool,
            default_value=False,
            comment="Forces the use of Tesla Pedal over ACC completely disabling the Tesla CC.",
        )
        file_changed |= didUpdate

        # enable_hso -> enableHSO
        into.enableHSO, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="enable_hso",
            entry_type=bool,
            default_value=True,
            comment="Enables Human Steering Override (HSO) feature which allows you to take control of the steering wheel and correct the course of the car without disengaging OpenPilot lane keep assist (LKS, lateral control).",
        )
        file_changed |= didUpdate

        # enable_hao -> enableHAO
        into.enableHAO, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="enable_hao",
            entry_type=bool,
            default_value=True,
            comment="Enables Human Acceleration Override (HAO) feature which allows you to take control of the acceleration without disengaging OpenPilot lane keep assist (LKS, lateral control).",
        )
        file_changed |= didUpdate

        # auto_start_alca_delay -> autoStartAlcaDelay
        into.autoStartAlcaDelay, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="auto_start_alca_delay",
            entry_type=int,
            default_value=2,
            comment="Set this setting to a value greater than 1 if you want lane change to start automatically after x seconds.",
        )
        file_changed |= didUpdate

        # enable_radar_emulation -> enableRadarEmulation
        into.enableRadarEmulation, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="enable_radar_emulation",
            entry_type=bool,
            default_value=False,
            comment="The secret sauce to make the Tesla Radar work; this feature makes the Panda generate all the CAN messages needed by the Tesla Bosch Radar to operate.",
        )
        file_changed |= didUpdate

        # use_tesla_radar -> useTeslaRadar
        into.useTeslaRadar, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="use_tesla_radar",
            entry_type=bool,
            default_value=False,
            comment="Set this setting to True if you have a Tesla Bosch Radar installed (works in conjunction with enable_radar_emulation).",
        )
        file_changed |= didUpdate

        # radar_vin -> into.radarVIN
        default_radar_vin = '"                 "'
        into.radarVIN, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="radar_vin",
            entry_type=str,
            default_value=default_radar_vin,
            comment="If you use an aftermarket Tesla Bosch Radar that already has a coded VIN, you will have to enter that VIN value here.",
        )
        file_changed |= didUpdate

        if into.radarVIN == "":
            into.radarVIN = default_radar_vin
            file_changed = True

        # radar_offset -> radarOffset
        into.radarOffset, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="radar_offset",
            entry_type=float,
            default_value=0,
            comment="If your Tesla Bosch Radar is not centered on the car, this value will allow to enter a correction offset.",
        )
        file_changed |= didUpdate

        # radar_epas_type -> radarEpasType
        into.radarEpasType, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="radar_epas_type",
            entry_type=int,
            default_value=0,
            comment="Depending on the source of your Tesla Bosch Radar (older or newer Model S or Model X), this setting has to match what the radar was programmed to recognize as EPAS; values are between 0 and 4; finding the right one is trial and error.",
        )
        file_changed |= didUpdate

        # radar_position -> radarPosition
        into.radarPosition, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=main_section,
            entry="radar_position",
            entry_type=int,
            default_value=0,
            comment="Depending on the source of your Tesla Bosch Radar (older or newer Model S or Model X), this setting has to match what the radar was programmed to have a position (Model S, Model S facelift, Model X); values are between 0 and 3; finding the right one is trial and error.",
        )
        file_changed |= didUpdate

        # hso_numb_period -> hsoNumbPeriod
        into.hsoNumbPeriod, didUpdate = self.read_config_entry(
            config,
            configr,
            prev_file_contents,
            section=pref_section,
            entry="hso_numb_period",
            entry_type=float,
            default_value=1.5,
            comment="Seconds to delay the reengagement of LKAS after turn signal has been used. Time starts when the turn signal is turned on.",
        )
        file_changed |= didUpdate

        if file_changed:
            did_write = True
            with open(config_path, self.config_file_w) as configfile:
                config.write(configfile)
        else:
            did_write = False

        # Remove double quotes from VIN (they are required for empty case)
        into.radarVIN = into.radarVIN.replace('"', "")
        return did_write

    def read_config_entry(
        self,
        config,
        configr,
        prev_file_contents,
        section,
        entry,
        entry_type,
        default_value,
        comment,
    ):
        updated = self.update_comment(
            config, prev_file_contents, section, entry, default_value, comment
        )
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

    def update_comment(
        self, config, prev_file_contents, section, entry, default_value, comment
    ):
        new_comment = (
            "# " + entry + ": " + comment + " (Default: " + str(default_value) + ")"
        ).lower()
        config.set(section, new_comment)
        updated = prev_file_contents.find(new_comment) == -1
        return updated


class CarSettings:
    forcePedalOverCC = None
    enableHSO = None
    enableRadarEmulation = None
    autoStartAlcaDelay = None
    useTeslaRadar = None
    radarVIN = None
    radarOffset = None
    radarEpasType = None
    radarPosition = None
    hsoNumbPeriod = None
    usesApillarHarness = None
    enableHAO = None

    def __init__(self, optional_config_file_path=default_config_file_path):
        config_file = ConfigFile()
        self.did_write_file = config_file.read(
            self, config_path=optional_config_file_path
        )

    def get_value(self, name_of_variable):
        return self.__dict__[name_of_variable]


# Legacy support
def read_config_file(into, config_path=default_config_file_path):
    config_file = ConfigFile()
    config_file.read(into, config_path)