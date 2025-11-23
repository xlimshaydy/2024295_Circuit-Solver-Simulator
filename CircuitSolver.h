#ifndef CIRCUIT_SOLVER_H
#define CIRCUIT_SOLVER_H

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>   // For unique_ptr
#include <fstream>  // For File Handling
#include <cmath>    // For math operations
#include <iomanip>  // For output formatting
#include <stdexcept> // Needed for error handling

using namespace std;

// Enum to identify component types easily
enum ComponentType {
    RESISTOR,
    CURRENT_SOURCE,
    VOLTAGE_SOURCE
};

// ==========================================
// 1. Component Classes (Inheritance/Polymorphism)
// ==========================================

// Base Class for all components
class Component {
public:
    string name;
    int nodeA_ID; // Internal integer ID for the first node
    int nodeB_ID; // Internal integer ID for the second node
    double value; // Resistance (Ohms) or Current (Amps) or Voltage (Volts)

    // Virtual destructor ensures derived classes are cleaned up correctly
    virtual ~Component() {}

    // Pure virtual function: every component must define its own type
    virtual ComponentType getType() const = 0;

    // Virtual function to print component details
    virtual void display() const {
        cout << "Component: " << name << " | Nodes: " << nodeA_ID << " - " << nodeB_ID 
             << " | Value: " << value << endl;
    }
};

// Resistor Class
class Resistor : public Component {
public:
    Resistor(string n, int nA, int nB, double r) {
        // Error Check: Resistance must be positive
        if (r <= 0) {
            throw invalid_argument("Error: Resistance must be positive.");
        }
        // Error Check: Cannot connect to same node
        if (nA == nB) {
            throw invalid_argument("Error: Resistor '" + n + "' cannot be connected to the same node.");
        }

        name = n;
        nodeA_ID = nA;
        nodeB_ID = nB;
        value = r;
    }

    ComponentType getType() const override { return RESISTOR; }
    
    // Helper to get Conductance (G = 1/R)
    double getConductance() const {
        if (value == 0) return 0.0; // Should be caught by constructor, but safe to keep
        return 1.0 / value;
    }
};

// Current Source Class
class CurrentSource : public Component {
public:
    CurrentSource(string n, int nA, int nB, double i) {
        if (nA == nB) {
            throw invalid_argument("Error: Current Source '" + n + "' cannot be connected to the same node.");
        }
        name = n;
        nodeA_ID = nA;
        nodeB_ID = nB;
        value = i;
    }

    ComponentType getType() const override { return CURRENT_SOURCE; }
};

// Voltage Source Class
class VoltageSource : public Component {
public:
    VoltageSource(string n, int nA, int nB, double v) {
        if (nA == nB) {
            throw invalid_argument("Error: Voltage Source '" + n + "' cannot be connected to the same node.");
        }
        name = n;
        nodeA_ID = nA;
        nodeB_ID = nB;
        value = v;
    }

    ComponentType getType() const override { return VOLTAGE_SOURCE; }
};

// ==========================================
// 2. Circuit Manager Class (The "Graph")
// ==========================================

class Circuit {
private:
    // List of all components in the circuit (using smart pointers for safety)
    vector<unique_ptr<Component>> components;

    // Hash Map to link user-friendly names ("Vout") to internal IDs (0, 1, 2)
    // Key: Node Name (string), Value: Matrix Index (int)
    unordered_map<string, int> nodeName_to_ID;

    // Store results: Node ID -> Voltage Value
    unordered_map<int, double> nodeVoltages;

    int nodeCount = 0; // Counter for unique nodes assigned

public:
    // Constructor
    Circuit() {
        // Always reserve ID 0 for Ground (GND)
        nodeName_to_ID["GND"] = 0;
        nodeName_to_ID["0"] = 0;
        nodeVoltages[0] = 0.0; // Ground is always 0V
    }

    // --- Feature: Dynamic Circuit Creation ---
    
    // Helper to get or create a node ID from a string name
    int getNodeID(const string& nodeName) {
        if (nodeName.empty()) {
            throw invalid_argument("Error: Node name cannot be empty.");
        }
        if (nodeName_to_ID.find(nodeName) == nodeName_to_ID.end()) {
            // New node found
            nodeCount++;
            nodeName_to_ID[nodeName] = nodeCount;
            return nodeCount;
        }
        return nodeName_to_ID[nodeName];
    }

    void addResistor(string name, string n1, string n2, double resistance) {
        int id1 = getNodeID(n1);
        int id2 = getNodeID(n2);
        // Validation happens inside Resistor constructor
        components.push_back(make_unique<Resistor>(name, id1, id2, resistance));
    }

    void addCurrentSource(string name, string nFrom, string nTo, double current) {
        int id1 = getNodeID(nFrom);
        int id2 = getNodeID(nTo);
        components.push_back(make_unique<CurrentSource>(name, id1, id2, current));
    }

    void addVoltageSource(string name, string nPos, string nNeg, double voltage) {
        int id1 = getNodeID(nPos);
        int id2 = getNodeID(nNeg);
        components.push_back(make_unique<VoltageSource>(name, id1, id2, voltage));
    }
    
    void clearCircuit() {
        components.clear();
        nodeName_to_ID.clear();
        nodeVoltages.clear();
        nodeCount = 0;
        // Re-initialize ground
        nodeName_to_ID["GND"] = 0;
        nodeVoltages[0] = 0.0;
    }

    // --- Feature: Nodal Analysis Solver ---
    void solve();

    // --- Feature: Results Display ---
    void displayResults(); // Implementation is in .cpp

    // --- Feature: File I/O (Save/Load) ---
    void saveCircuit(const string& filename); // Implementation is in .cpp
    void loadCircuit(const string& filename);

    // --- Feature: Visualization ---
    void visualizeCircuit();
    void exportGraphviz();
};

#endif // CIRCUIT_SOLVER_H