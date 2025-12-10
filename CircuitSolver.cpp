#include "CircuitSolver.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <algorithm> 

using namespace std;


// Helper: Numerical Sort Comparator


bool compareNodes(const pair<string, int>& a, const pair<string, int>& b) {
    string s1 = a.first;
    string s2 = b.first;

    // Check if both are purely numeric
    bool isNum1 = !s1.empty() && std::all_of(s1.begin(), s1.end(), ::isdigit);
    bool isNum2 = !s2.empty() && std::all_of(s2.begin(), s2.end(), ::isdigit);

    if (isNum1 && isNum2) {
        // If lengths differ, the shorter number is smaller (e.g. "2" < "10")
        if (s1.length() != s2.length()) {
            return s1.length() < s2.length();
        }
    }
    // Otherwise use standard alphabetical sort (handles "10" vs "20" or "n1" vs "n2")
    return s1 < s2;
}

// Helper: Robust Gaussian Elimination

vector<double> gaussianElimination(vector<vector<double>> A, vector<double> B) {
    int n = A.size();
    const double EPSILON = 1e-9;
    
    for (int i = 0; i < n; i++) {
        double maxEl = abs(A[i][i]);
        int maxRow = i;
        for (int k = i + 1; k < n; k++) {
            if (abs(A[k][i]) > maxEl) {
                maxEl = abs(A[k][i]);
                maxRow = k;
            }
        }
        swap(A[maxRow], A[i]);
        swap(B[maxRow], B[i]);

        if (abs(A[i][i]) < EPSILON) {
            throw runtime_error("Singular Matrix detected! The circuit may have floating nodes, no ground reference, or invalid loops.");
        }

        for (int k = i + 1; k < n; k++) {
            double factor = A[k][i] / A[i][i];
            B[k] -= factor * B[i];
            for (int j = i; j < n; j++) {
                A[k][j] -= factor * A[i][j];
            }
        }
    }

    vector<double> x(n);
    for (int i = n - 1; i >= 0; i--) {
        double sum = 0;
        for (int j = i + 1; j < n; j++) {
            sum += A[i][j] * x[j];
        }
        x[i] = (B[i] - sum) / A[i][i];
    }
    return x;
}

// Circuit::solve() Implementation
void Circuit::solve() {
    try {
        if (nodeCount == 0) throw runtime_error("Circuit is empty. Add components first.");

        // PRE-CHECK: Ensure at least one component connects to Ground (ID 0)
        bool groundConnected = false;
        for (const auto& comp : components) {
            if (comp->nodeA_ID == 0 || comp->nodeB_ID == 0) {
                groundConnected = true;
                break;
            }
        }
        if (!groundConnected) {
            throw runtime_error("No Ground reference! At least one component must connect to node '0' or 'GND'.");
        }

        int vSourceCount = 0;
        for (const auto& comp : components) if (comp->getType() == VOLTAGE_SOURCE) vSourceCount++;
        int matrixSize = nodeCount + vSourceCount;

        vector<vector<double>> A(matrixSize, vector<double>(matrixSize, 0.0));
        vector<double> B(matrixSize, 0.0);
        int vSourceIndex = 0;

        cout << "Building MNA System (" << matrixSize << "x" << matrixSize << ")..." << endl;

        for (const auto& comp : components) {
            if (comp->getType() == RESISTOR) {
                Resistor* r = static_cast<Resistor*>(comp.get());
                double g = r->getConductance();
                int u = r->nodeA_ID, v = r->nodeB_ID;
                if (u!=0) { A[u-1][u-1]+=g; if(v!=0) A[u-1][v-1]-=g; }
                if (v!=0) { A[v-1][v-1]+=g; if(u!=0) A[v-1][u-1]-=g; }
            }
            else if (comp->getType() == CURRENT_SOURCE) {
                CurrentSource* cs = static_cast<CurrentSource*>(comp.get());
                int u = cs->nodeA_ID, v = cs->nodeB_ID;
                if (u!=0) B[u-1] -= cs->value;
                if (v!=0) B[v-1] += cs->value;
            }
            else if (comp->getType() == VOLTAGE_SOURCE) {
                VoltageSource* vs = static_cast<VoltageSource*>(comp.get());
                int rIdx = nodeCount + vSourceIndex;
                int p = vs->nodeA_ID, n = vs->nodeB_ID;
                if (p!=0) { A[p-1][rIdx] = 1; A[rIdx][p-1] = 1; }
                if (n!=0) { A[n-1][rIdx] = -1; A[rIdx][n-1] = -1; }
                B[rIdx] = vs->value;
                vSourceIndex++;
            }
        }

        vector<double> result = gaussianElimination(A, B);
        
        // Only update voltages if solver succeeded
        for (int i = 0; i < nodeCount; i++) nodeVoltages[i + 1] = result[i];
        cout << "Circuit Solved Successfully!" << endl;

    } catch (const exception& e) {
        cerr << "\n[SOLVER ERROR]: " << e.what() << "\n" << endl;
    }
}

// Circuit::loadCircuit() Implementation
void Circuit::loadCircuit(const string& filename) {
    ifstream inFile(filename);
    if (!inFile) {
        cerr << "Error: Could not open file " << filename << "\n";
        return;
    }
    try {
        clearCircuit();
        nodeName_to_ID["0"] = 0;
        nodeName_to_ID["GND"] = 0;
        nodeName_to_ID["gnd"] = 0;

        string line, type, name, n1, n2;
        double val;
        int count = 0;
        while (getline(inFile, line)) {
            if (line.empty()) continue;
            stringstream ss(line);
            if (!(ss >> type >> name >> n1 >> n2 >> val)) {
                 cerr << "Warning: Skipping malformed line: " << line << endl;
                 continue;
            }

            if (type == "R" || type == "r") addResistor(name, n1, n2, val);
            else if (type == "I" || type == "i") addCurrentSource(name, n1, n2, val);
            else if (type == "V" || type == "v") addVoltageSource(name, n1, n2, val);
            else continue;
            count++;
        }
        cout << "Loaded " << count << " components.\n";
    } catch (const exception& e) {
        cerr << "Error loading: " << e.what() << endl;
        clearCircuit();
    }
    inFile.close();
}


// Circuit::displayResults() - FIXED & NUMERICALLY SORTED

void Circuit::displayResults() {
    if (nodeVoltages.size() <= 1) {
        cout << "No results available. Please solve the circuit first.\n";
        return;
    }

    // Step 1: Store {NodeName, NodeID} in a vector
    vector<pair<string, int>> sortedNodes;
    for (const auto& pair : nodeName_to_ID) {
        sortedNodes.push_back({pair.first, pair.second});
    }

    // Step 2: Sort using Custom Numerical Comparator
    sort(sortedNodes.begin(), sortedNodes.end(), compareNodes);

    cout << "\n--- Simulation Results ---\n";
    for (const auto& pair : sortedNodes) {
        string name = pair.first;
        int id = pair.second;
        
        if (id == 0) continue; 

        if (nodeVoltages.find(id) != nodeVoltages.end()) {
            cout << "Node [" << name << "]: " 
                 << fixed << setprecision(3) << nodeVoltages[id] << " V" << endl;
        }
    }
    cout << "--------------------------\n";
}

// Circuit::saveCircuit()

void Circuit::saveCircuit(const string& filename) {
    ofstream outFile(filename);
    if (!outFile.is_open()) {
        cerr << "Error: Could not save to file " << filename << endl;
        return;
    }

    unordered_map<int, string> id_to_name;
    for (const auto& pair : nodeName_to_ID) {
        id_to_name[pair.second] = pair.first;
    }

    for (const auto& comp : components) {
         char typeChar = 'R';
         if (comp->getType() == CURRENT_SOURCE) typeChar = 'I';
         if (comp->getType() == VOLTAGE_SOURCE) typeChar = 'V';
         
         string nA = id_to_name[comp->nodeA_ID];
         string nB = id_to_name[comp->nodeB_ID];

         outFile << typeChar << " " << comp->name << " " 
                 << nA << " " << nB << " " 
                 << comp->value << "\n";
    }
    outFile.close();
    cout << "Circuit saved to " << filename << endl;
}


// Circuit::visualizeCircuit()

void Circuit::visualizeCircuit() {
    if (nodeName_to_ID.empty()) {
        cout << "Circuit is empty. Nothing to visualize.\n";
        return;
    }

    cout << "\n====== CIRCUIT GRAPH TOPOLOGY (Adjacency List) ======\n";
    
    // Sort names numerically here too
    vector<pair<string, int>> sortedNodes;
    for (const auto& pair : nodeName_to_ID) sortedNodes.push_back({pair.first, pair.second});
    sort(sortedNodes.begin(), sortedNodes.end(), compareNodes);

    for (const auto& pair : sortedNodes) {
        string currentNodeName = pair.first;
        int currentNodeID = pair.second;

        cout << " Node [" << currentNodeName << "] connects to:\n";

        bool hasConnection = false;
        
        for (const auto& comp : components) {
            int neighborID = -1;
            string arrow = "";

            if (comp->nodeA_ID == currentNodeID) {
                neighborID = comp->nodeB_ID;
                arrow = " --- "; 
                if (comp->getType() == CURRENT_SOURCE) arrow = " --> "; 
                if (comp->getType() == VOLTAGE_SOURCE) arrow = " (+)- ";
            } 
            else if (comp->nodeB_ID == currentNodeID) {
                neighborID = comp->nodeA_ID;
                arrow = " --- ";
                if (comp->getType() == CURRENT_SOURCE) arrow = " <-- "; 
                if (comp->getType() == VOLTAGE_SOURCE) arrow = " -(-) ";
            }

            if (neighborID != -1) {
                hasConnection = true;
                string neighborName = "Unknown";
                for(const auto& p : nodeName_to_ID) { 
                    if(p.second == neighborID) { neighborName = p.first; break; } 
                }

                cout << "   |-- [" << comp->name << " (" << comp->value << ")]" 
                     << arrow << " Node [" << neighborName << "]\n";
            }
        }

        if (!hasConnection) cout << "   (No connections - Isolated)\n";
        cout << "\n";
    }
    cout << "=====================================================\n";
}

