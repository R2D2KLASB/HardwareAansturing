# HardwareAansturing
Dit is de hardware aansturing voor een xy-plotter.
Voor de codestandaard, klik [hier](https://github.com/R2D2KLASB/Info/blob/main/CodeStandaard.md)


# Speaker
Om de speaker te kunnen gebruiken is het nodig om een library te installeren. via de command line moet het volgende commando uitgevoerd worden: 

`sudo apt install ffmpeg`

Wanneer dit geinstalleerd is en er is een speaker aangesloten op de headphone uitgang op de RPI zullen er geluiden afspelen tijdens het spelen van de game.


# Arduino plotter
Om de seriele verbinding tussen de RPI te gebruiken moet er een USB kabel vanaf de RPI naar de ***programming*** port van de arduino due waar de header van de xy plotter op aamgesloten zit.

# Arduino display
Om de seriele verbinding tussen de RPI te gebruiken moet er een USB kabel vanaf de RPI naar de ***native*** port van de arduino due waar de header van het display op aangesloten zit.

# LibSerial
Om de seriele communicatie te gebruiken is er gebruik gemaamkt van LibSerial, om deze library te installeren via de command line het volgende commando uiitgevoerd worden: 

`sudo apt install libserial-dev`

Wanner deze is geinstalleerd en de USB kabels op de juiste poorten zijn aangesloten werkt de seriele communicatie met de RPI en de arduino's.
