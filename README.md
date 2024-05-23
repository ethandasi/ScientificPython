# ScientificPython

## Cahier des charges

Étant donné un environnement 2D plan muni de trois amers parfaitement distinguables préalablement localisées de manière absolue,

1. en parallèle à 2 ci-dessous : localiser séquentiellement un robot mobile non holonome en mouvement sur la base de (1) son odométrie ; (2) la perception des coordonnées cartésiennes relatives de chacun de ces amers (ou bien du couple azimut-distance relatif à chaque amer) depuis le robot ; le volet proprioperception et perception des amers serait réduit à la simulation des déplacements et des mesures relatives aux amers (pas de technique de computer vision, analyse d’image, détection-étiquetage d’amers, odométrie optique, etc.) ; le moteur de localisation serait le filtre de Kalman (extension non linéaire) ;

2. en parallèle à 1 ci-dessus : commander un robot mobile non holonome que l’on suppose localisé exacte de manière absolue (position2D et orientation absolues) de telle sorte qu’il s’asservisse sur la trajectoire (chemin + loi horaire) d’un robot de référence ; techniques de commande de robots mobiles ;

3. faire se rejoindre les points 1 et 2 ci-dessus de telle sorte que l’asservissement sur une trajectoire de référence s’effectue sur la localisation obtenue obtenue par Kalman au lieu d’une supposée localisation exacte.
