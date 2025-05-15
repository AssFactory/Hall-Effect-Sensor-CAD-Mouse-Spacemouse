Import("env")
board_config = env.BoardConfig()
board_config.update("build.hwids", [
  ["0x256f", "0xc631"]
])
board_config.update("build.usb_product", "Spacemouse")
board_config.update("vendor", "ASS Factory")