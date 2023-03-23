from panda import Panda

p = Panda()

p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

p.enter_bootloader()
time.sleep(1)
print("done sending bootloader mode command")
exit(0)