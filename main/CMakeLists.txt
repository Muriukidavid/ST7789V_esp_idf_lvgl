set(srcs "main.c"
    "pretty_effect.c"
    "decode_image.c"
    "jpeg_decoder.c"
    )

idf_component_register(SRCS ${srcs}
                INCLUDE_DIRS "."
                EMBED_FILES image.jpg)