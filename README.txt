1. Πετρος Χονδρογιαννης sdi2100254	
2.Εντολή Μεταγλώττισης (Compilation) : g++ -std=c++17 main.cpp GridWorld.cpp WorldObject.cpp StaticObjects.cpp MovingObjects.cpp Sensors.cpp Vehicle.cpp Visualization.cpp -o simulation
3. Εντολή Εκτέλεσης (Execution) : ./simulation --dimX 100 --dimY 20 --gps 10 5 30 5 50 10 80 15 --simulationTicks 100 --numMovingCars 5 --numStopSigns 2
	Διαθέσιμες Παράμετροι:

	--dimX <int>, --dimY <int>: Διαστάσεις του πλέγματος (Grid).

	--gps <x> <y> ...: Ζεύγη συντεταγμένων που ορίζουν την πορεία του αυτόνομου οχήματος (Υποχρεωτικό).

	--simulationTicks <int>: Διάρκεια προσομοίωσης σε βήματα.

	--numMovingCars <int>, --numMovingBikes <int>: Αριθμός κινούμενων οχημάτων.

	--numParkedCars <int>, --numStopSigns <int>, --numTrafficLights <int>: Αριθμός στατικών αντικειμένων.

	--minConfidenceThreshold <double>: Ελάχιστο όριο εμπιστοσύνης αισθητήρων.

	--seed <uint>: Seed για την τυχαία παραγωγή του κόσμου (για επαναληψιμότητα).

4.Ο κόσμος είναι ένα δισδιάστατο πλέγμα (Grid) διακριτών συντεταγμένων.

Ο χρόνος είναι διακριτός και μετριέται σε "ticks". Σε κάθε tick συμβαίνει μια ενημέρωση όλων των οντοτήτων.

Η όραση του οχήματος και η λήψη αποφάσεων εξαρτώνται από το minConfidenceThreshold. Αντικείμενα με χαμηλή πιθανότητα ανίχνευσης αγνοούνται.

