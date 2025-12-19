#include <iostream>
#include <string>
#include <limits>
#include "CircuitSolver.h"
using namespace std;

// Helper to prevent crashes on invalid input
double getValidDouble(const string& prompt) {
    double value;
    while (true) {
        cout << prompt;
        if (cin >> value) {
            return value;
        } else {
            cout << "Invalid input. Please enter a number.\n";
            cin.clear(); 
            cin.ignore(numeric_limits<streamsize>::max(), '\n'); 
        }
    }
}

void printMenu() {
    cout << "\n========================================\n";
    cout << "   C++ CIRCUIT SOLVER (MNA Algorithm)   \n";
    cout << "========================================\n";
    cout << "1. Add Resistor\n";
    cout << "2. Add Current Source\n";
    cout << "3. Add Voltage Source\n";
    cout << "4. Solve Circuit\n";
    cout << "5. Save Circuit\n";
    cout << "6. Load Circuit (Auto-Solves)\n"; 
    cout << "7. Clear Circuit\n";
    cout << "8. Visualize Circuit (Text Graph)\n";
    cout << "0. Exit\n";
    cout << "========================================\n";
    cout << "Enter choice: ";
}

int main() {
    Circuit circuit;
    int choice;
    string name, n1, n2, filename;
    double value;

    while (true) {
        printMenu();
        
        if (!(cin >> choice)) {
            cout << "Invalid input. Please enter a number (0-9).\n";
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            continue;
        }

        try {
            switch (choice) {
            case 1:
                cout << "Enter Name (e.g., R1): "; cin >> name;
                cout << "Enter Node A: "; cin >> n1;
                cout << "Enter Node B: "; cin >> n2;
                value = getValidDouble("Enter Resistance (Ohms): ");
                circuit.addResistor(name, n1, n2, value);
                cout << "Resistor added.\n";
                break;

            case 2:
                cout << "Enter Name (e.g., I1): "; cin >> name;
                cout << "Enter Node From: "; cin >> n1;
                cout << "Enter Node To: "; cin >> n2;
                value = getValidDouble("Enter Current (Amps): ");
                circuit.addCurrentSource(name, n1, n2, value);
                cout << "Current Source added.\n";
                break;

            case 3:
                cout << "Enter Name (e.g., V1): "; cin >> name;
                cout << "Enter Positive Node: "; cin >> n1;
                cout << "Enter Negative Node: "; cin >> n2;
                value = getValidDouble("Enter Voltage (Volts): ");
                circuit.addVoltageSource(name, n1, n2, value);
                cout << "Voltage Source added.\n";
                break;

            case 4:
                circuit.solve();
                circuit.displayResults();
                break;

            case 5:
                cout << "Enter filename to save: "; cin >> filename;
                circuit.saveCircuit(filename);
                break;

            case 6: // UPDATED: Auto-Solve after loading
                cout << "Enter filename to load: "; cin >> filename;
                circuit.loadCircuit(filename);
                // Automatically solve and show results to the user
                cout << "Auto-solving loaded circuit...\n";
                circuit.solve();
                circuit.displayResults();
                break;

            case 7:
                circuit.clearCircuit();
                cout << "Circuit cleared.\n";
                break;
            
            case 8:
                circuit.visualizeCircuit();
                break;
            

            case 0:
                cout << "Exiting.\n";
                return 0;

            default:
                cout << "Invalid choice. Try again.\n";
            }
        }
        catch (const exception& e) {
            cout << "\n[ERROR]: " << e.what() << "\n";
        }
    }
    return 0;
}
//g++ CircuitSolver.cpp  main.cpp -o main.exe makes a file
// .\main.exe
// cd "DSA Project"
// dir