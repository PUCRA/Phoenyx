import time
import board
import neopixel

class LEDS:
    def __init__(self):
        # Usamos GPIO10 y la configuración para RGB (WS2812B)
        self.pixel_pin = board.D10
        self.num_pixels = 5
        self.ORDER = neopixel.RGB  # WS2812B usa RGB, no RGBW
        # Desactivar auto_write para poder controlar manualmente cuando se actualizan los LEDs
        self.pixels = neopixel.NeoPixel(
            self.pixel_pin, self.num_pixels, pixel_order=self.ORDER, auto_write=True
        )
        time.sleep(5)

    def set_color(self, color):
        print(f"Setting color: {color}")
        
        # Establece el color de todos los LEDs
        self.pixels.fill(color)
        # Muestra los cambios
        self.pixels.show()

    def clear(self):
        # Apaga todos los LEDs
        self.pixels.fill((0, 0, 0))  # Apaga todos los LEDs (RGB)
        self.pixels.show()  # Muestra los cambios

def main():
    leds = LEDS()
    
    print("Iniciando LEDs")
    leds.clear()  # Apaga los LEDs al principio
    time.sleep(1)
    
    # Establecer colores
    leds.set_color((0, 0, 0))  # Rojo
    time.sleep(1)
    
    leds.set_color((0, 255, 0))  # Verde
    time.sleep(1)
    
    leds.set_color((0, 0, 255))  # Azul
    time.sleep(1)
    
    leds.clear()  # Apaga los LEDs al final
    time.sleep(1)

if __name__ == "__main__":
    main()
