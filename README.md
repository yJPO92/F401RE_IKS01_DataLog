\mainpage
# F401RE\_IKS01\_Unicleo
Nucleo F401RE + shield IKSA3 + datalogging on VT or via Unicleo_GUI.

### How it works
Download binary (.bin) into Nucleo board.

Start Unicleo-GUI, follow instruction.

Stop&close Unicleo-Gui, start a terminal (i.e. TeraTerm), press BP1.

_Nota_: stop terminal before use Unicleo-Gui.

## Getting Started
### How is made
Start from sample: STMicroelectronics\X-CUBE-MEMS1\6.2.0\Projects\STM32F401RE-Nucleo\Examples\IKS01A3\DataLogExtended ; and mix with \DataLogTerminal.

Include switch between VT terminal & Unicleo-GUI with BP1.

Quelques modifs manuelles ds les fichiers pour mixer les 2 examples.

_Nota_: STM specifics folders (and sub-folder) not included in this repository:

\Drivers\BSP

\Drivers\CMSIS

\Drivers\STM32F4xx_HAL_Driver

## Contributing
NA.
But you can fork it.

## Versioning
For the versions available, see the [F401RE_IKS01_Unicleo](https://github.com/yJPO92/F401RE_IKS01_Unicleo).

## Authors
* **STMicroelectronics** - *Initial projet*
* **JPO** - *Adaptation*

## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

Refer, also, to STMicroelectronics license [STM-readme-DataLogExtended.txt](STM-readme-DataLogExtended.txt) & [STM-readme-DataLogTerminal.txt](STM-readme-DataLogTerminal.txt).


