# CMakeLists.txt en ~/esp/esp-idf/examples/camera_example/main

# Establece el nombre del componente
idf_component_register(SRCS "take_picture.c"
                    INCLUDE_DIRS "."
                    PRIV_INCLUDE_DIRS "."
                    PRIV_REQUIRES nvs_flash tensorflow)  # Agrega tensorflow aquí

# Si el archivo model_1.h está en una carpeta llamada 'models', descomenta la siguiente línea:
# include_directories("models")
