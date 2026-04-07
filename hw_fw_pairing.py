from SDECv2.BaseController import BaseController, create_controllers, create_firmwares

fc_rev2_appa = BaseController(
    controller=create_controllers.flight_computer_rev2_controller(),
    firmware=create_firmwares.appa_firmware()
)

gs_rev1_receiver = BaseController(
    controller=create_controllers.ground_station_rev1_controller(),
    firmware=create_firmwares.receiver_firmware()
)

HW_FW_PAIRS = [
    fc_rev2_appa,
    gs_rev1_receiver
] 