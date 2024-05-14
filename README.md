# QuanshengPress

The objective of this project is to automate the operation of Quansheng UV-K5 (handheld transceiver), especially frequency changes using rigctl interface. **It requires firmware modification** [Quansheng Dock FW](https://github.com/nicsure/quansheng-dock-fw) and leverages it's (virtual) keypress emulation.

> The code is in development phase. Basic usage you will find when you run the script without any parameter. This README will be updated when [minimum viable product](https://en.wikipedia.org/wiki/Minimum_viable_product) is ready. It will be also announced in Facebook group [Quansheng Dock/Xvfo UV- K5/K6/5R+](https://www.facebook.com/groups/289656334131909/posts/363009730129902/).

[Gpredict](https://github.com/csete/gpredict) Radio Interface configuration:

![gpredict](https://github.com/BranoSundancer/QuanshengPress/assets/127756743/03e5bb07-2f7c-45d9-a4c6-5c66f36b76c2)

For satellite tracking with frequent frequency changes there is better (more stable & reliable, faster, PTT-aware) approach for this radio: [Quansheng Dock (**mod/om1atb**)](https://github.com/BranoSundancer/QuanshengDock-mod-om1atb/releases), which can be connected to [Hamlib](https://github.com/Hamlib/Hamlib)'s _rotctld_ as FT-991 (CAT COM interface) as presented in this [Facebook post](https://www.facebook.com/groups/289656334131909/posts/364416183322590/).
