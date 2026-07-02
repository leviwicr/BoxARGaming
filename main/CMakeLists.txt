idf_component_register(SRCS "main.c"
                         "camera/camera_driver.c"
                         "detection/detection_driver.cpp"
                    INCLUDE_DIRS "."
                                 "camera"
                                 "detection"
                    REQUIRES esp_video coco_detect)
