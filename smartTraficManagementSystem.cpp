#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <thread>
#include <mutex>
#include <chrono>
#include <memory>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <condition_variable>
#include <string>
#include <cmath>
#include <limits> // for std::numeric_limits

//======================================================================
// 1. Observer Pattern Interfaces
//======================================================================

// Forward declaration
class TrafficSignal;

/**
 * @brief Observer interface that vehicles implement to get updates from TrafficSignal.
 */
class ITrafficSignalObserver
{
public:
    virtual ~ITrafficSignalObserver() = default;
    virtual void onSignalChange(const TrafficSignal &signal) = 0;
};

//======================================================================
// 2. TrafficSignal (Subject in the Observer Pattern)
//======================================================================

/**
 * @brief Represents a traffic signal which notifies observers (vehicles).
 */
class TrafficSignal
{
public:
    enum class LightColor
    {
        RED,
        GREEN,
        YELLOW
    };

private:
    LightColor color;
    std::vector<ITrafficSignalObserver *> observers;

public:
    TrafficSignal() : color(LightColor::RED) {}

    void attach(ITrafficSignalObserver *obs)
    {
        observers.push_back(obs);
    }

    void detach(ITrafficSignalObserver *obs)
    {
        observers.erase(
            std::remove(observers.begin(), observers.end(), obs),
            observers.end());
    }

    void setColor(LightColor newColor)
    {
        color = newColor;
        notifyObservers();
    }

    LightColor getColor() const
    {
        return color;
    }

    void notifyObservers()
    {
        for (auto *obs : observers)
        {
            obs->onSignalChange(*this);
        }
    }
};

//======================================================================
// 3. Template Classes (Generic Programming) - PriorityQueue Example
//======================================================================

/**
 * @brief A simple priority queue that can handle different data types.
 *        Uses a comparison functor for custom ordering if needed.
 */
template <typename T, typename Compare = std::less<T>>
class PriorityQueue
{
private:
    std::priority_queue<T, std::vector<T>, Compare> pq;

public:
    void push(const T &item)
    {
        pq.push(item);
    }

    T top()
    {
        return pq.top();
    }

    void pop()
    {
        pq.pop();
    }

    bool empty() const
    {
        return pq.empty();
    }

    size_t size() const
    {
        return pq.size();
    }
};

//======================================================================
// 4. Vehicle Classes (OOP: Inheritance, Polymorphism, Encapsulation)
//======================================================================

/**
 * @brief Abstract base Vehicle class.
 */
class Vehicle : public ITrafficSignalObserver
{
protected:
    std::string id;
    double speed; // in some units
    double size;  // e.g., length in meters
    int priority; // lower number => higher priority?

public:
    // Constructor with encapsulation
    Vehicle(const std::string &vehicleID, double vehicleSpeed, double vehicleSize, int vehiclePriority)
        : id(vehicleID), speed(vehicleSpeed), size(vehicleSize), priority(vehiclePriority) {}

    virtual ~Vehicle() = default;

    // Getters / Setters (Encapsulation)
    std::string getID() const { return id; }
    double getSpeed() const { return speed; }
    double getSize() const { return size; }
    int getPriority() const { return priority; }

    void setSpeed(double s) { speed = s; }
    void setSize(double sz) { size = sz; }
    void setPriority(int p) { priority = p; }

    // Observer callback
    virtual void onSignalChange(const TrafficSignal &signal) override
    {
        // Basic reaction: if signal is RED, we might slow down or stop
        using LightColor = TrafficSignal::LightColor;
        if (signal.getColor() == LightColor::RED)
        {
            // For demonstration, we set speed to 0
            speed = 0.0;
            std::cout << "[Vehicle " << id << "] RED signal => stopping.\n";
        }
        else if (signal.getColor() == LightColor::GREEN)
        {
            // Speed up to default (for simplicity, let's pick a fixed value)
            if (speed == 0.0)
                speed = 30.0;
            std::cout << "[Vehicle " << id << "] GREEN signal => resuming.\n";
        }
        else if (signal.getColor() == LightColor::YELLOW)
        {
            // Possibly slow down
            speed *= 0.5;
            std::cout << "[Vehicle " << id << "] YELLOW signal => slowing down.\n";
        }
    }

    // A method for movement simulation
    virtual void move() = 0; // purely virtual

    // For sorting/priority queue usage: smaller priority => "greater" in queue
    bool operator<(const Vehicle &other) const
    {
        return priority > other.priority;
    }
};

/**
 * @brief Car class deriving from Vehicle.
 */
class Car : public Vehicle
{
public:
    Car(const std::string &id, double speed, double size, int priority)
        : Vehicle(id, speed, size, priority) {}

    void move() override
    {
        std::cout << "[Car " << this->id << "] Moving at speed " << this->speed << "\n";
    }
};

/**
 * @brief Bus class deriving from Vehicle.
 */
class Bus : public Vehicle
{
public:
    Bus(const std::string &id, double speed, double size, int priority)
        : Vehicle(id, speed, size, priority) {}

    void move() override
    {
        std::cout << "[Bus " << this->id << "] Moving at speed " << this->speed << "\n";
    }
};

/**
 * @brief Bike class deriving from Vehicle.
 */
class Bike : public Vehicle
{
public:
    Bike(const std::string &id, double speed, double size, int priority)
        : Vehicle(id, speed, size, priority) {}

    void move() override
    {
        std::cout << "[Bike " << this->id << "] Moving at speed " << this->speed << "\n";
    }
};

//======================================================================
// 5. Road and Intersection (Simplified Model)
//======================================================================
/**
 * @brief Represents a road segment connecting intersections.
 */
struct Road
{
    std::string from;
    std::string to;
    double distance; // for route calculation
};

/**
 * @brief Utility function to implement a simple Dijkstra-like shortest path algorithm.
 */
std::vector<std::string> findShortestPath(
    const std::map<std::string, std::vector<std::pair<std::string, double>>> &graph,
    const std::string &start,
    const std::string &goal)
{
    // Distances map
    std::map<std::string, double> dist;
    std::map<std::string, std::string> prev;

    // Initialize distances
    for (auto &kv : graph)
    {
        dist[kv.first] = std::numeric_limits<double>::infinity();
    }
    dist[start] = 0.0;

    // Priority queue for exploring nodes
    auto cmp = [&](const std::string &lhs, const std::string &rhs)
    {
        return dist[lhs] > dist[rhs];
    };
    std::priority_queue<std::string, std::vector<std::string>, decltype(cmp)> pq(cmp);
    pq.push(start);

    while (!pq.empty())
    {
        std::string current = pq.top();
        pq.pop();

        if (current == goal)
        {
            break;
        }

        // If the current node is not in graph, skip
        if (graph.find(current) == graph.end())
            continue;

        for (auto &neighbor : graph.at(current))
        {
            const auto &nextNode = neighbor.first;
            double edgeDist = neighbor.second;
            double newDist = dist[current] + edgeDist;
            if (newDist < dist[nextNode])
            {
                dist[nextNode] = newDist;
                prev[nextNode] = current;
                pq.push(nextNode);
            }
        }
    }

    // Reconstruct path
    std::vector<std::string> path;
    if (prev.find(goal) == prev.end() && start != goal)
    {
        // No path found
        return path;
    }

    for (std::string at = goal; !at.empty(); at = prev[at])
    {
        path.push_back(at);
        if (at == start)
            break;
        if (prev.find(at) == prev.end())
        {
            // no path
            path.clear();
            break;
        }
    }
    std::reverse(path.begin(), path.end());
    return path;
}

//======================================================================
// 6. Singleton Pattern - Central TrafficControlSystem
//======================================================================
class TrafficControlSystem
{
private:
    // Private constructor for singleton
    TrafficControlSystem() = default;

    // Keep track of traffic signals
    std::map<std::string, std::shared_ptr<TrafficSignal>> signals;

    // Container for all vehicles
    std::vector<std::shared_ptr<Vehicle>> vehicles;

public:
    TrafficControlSystem(const TrafficControlSystem &) = delete;
    TrafficControlSystem &operator=(const TrafficControlSystem &) = delete;

    static TrafficControlSystem &getInstance()
    {
        static TrafficControlSystem instance;
        return instance;
    }

    // Manage signals
    void addSignal(const std::string &intersectionID, std::shared_ptr<TrafficSignal> signal)
    {
        signals[intersectionID] = signal;
    }

    std::shared_ptr<TrafficSignal> getSignal(const std::string &intersectionID)
    {
        if (signals.find(intersectionID) != signals.end())
        {
            return signals[intersectionID];
        }
        return nullptr;
    }

    // Manage vehicles
    void registerVehicle(std::shared_ptr<Vehicle> v)
    {
        vehicles.push_back(v);
    }

    const std::vector<std::shared_ptr<Vehicle>> &getVehicles() const
    {
        return vehicles;
    }
};

//======================================================================
// 7. Concurrency - Signal Controller
//======================================================================
/**
 * @brief This function runs in a separate thread to cycle traffic signals.
 */
void signalController(std::shared_ptr<TrafficSignal> signal, bool &stopFlag,
                      std::mutex &mtx, std::condition_variable &cv)
{
    using LightColor = TrafficSignal::LightColor;
    LightColor colors[] = {LightColor::RED, LightColor::GREEN, LightColor::YELLOW};
    int index = 0;

    while (true)
    {
        {
            // Check stop condition
            std::unique_lock<std::mutex> lock(mtx);
            if (stopFlag)
                break;
        }
        // Cycle signal color
        signal->setColor(colors[index]);
        index = (index + 1) % 3;

        // Sleep for demonstration
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}

//======================================================================
// 8. Simulation Thread
//======================================================================
void simulationThread(bool &stopFlag, std::mutex &mtx, std::condition_variable &cv)
{
    auto &tcs = TrafficControlSystem::getInstance();

    while (true)
    {
        {
            // Check stop condition
            std::unique_lock<std::mutex> lock(mtx);
            if (stopFlag)
                break;
        }

        // Move all vehicles
        for (auto &v : tcs.getVehicles())
        {
            v->move();
        }

        // Sleep to simulate a time step
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

//======================================================================
// 9. File Handling (Reading Configuration + Writing Results)
//======================================================================

/**
 * @brief Reads roads configuration from a text file in format:
 *        FROM TO DISTANCE
 *        e.g.:
 *        A B 5.0
 *        B C 4.0
 */
std::vector<Road> readRoadsFromFile(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Could not open roads config file: " + filename);
    }

    std::vector<Road> roads;
    std::string from, to;
    double dist;
    while (file >> from >> to >> dist)
    {
        roads.push_back({from, to, dist});
    }
    file.close();
    return roads;
}

/**
 * @brief Write the shortest path results to an output file.
 */
void writePathToFile(const std::string &filename, const std::vector<std::string> &path)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file for writing: " << filename << "\n";
        return;
    }
    file << "Shortest Path:\n";
    if (path.empty())
    {
        file << "No path found.\n";
    }
    else
    {
        for (const auto &node : path)
        {
            file << node << " ";
        }
        file << "\n";
    }
    file.close();
}

//======================================================================
// 10. Main Function
//======================================================================
int main()
{
    try
    {
        // --------------------------------------------------------------------
        // 1. Initialize Singleton System
        // --------------------------------------------------------------------
        auto &trafficSystem = TrafficControlSystem::getInstance();

        // --------------------------------------------------------------------
        // 2. Create Traffic Signals
        // --------------------------------------------------------------------
        auto signalA = std::make_shared<TrafficSignal>();
        auto signalB = std::make_shared<TrafficSignal>();
        auto signalC = std::make_shared<TrafficSignal>();

        trafficSystem.addSignal("A", signalA);
        trafficSystem.addSignal("B", signalB);
        trafficSystem.addSignal("C", signalC);

        // --------------------------------------------------------------------
        // 3. Create Vehicles
        // --------------------------------------------------------------------
        std::shared_ptr<Vehicle> car1 = std::make_shared<Car>("Car1", 60.0, 4.0, 2);
        std::shared_ptr<Vehicle> bus1 = std::make_shared<Bus>("Bus1", 40.0, 8.0, 1);
        std::shared_ptr<Vehicle> bike1 = std::make_shared<Bike>("Bike1", 30.0, 2.0, 3);

        // Attach vehicles to relevant signals (Observer pattern)
        signalA->attach(car1.get());
        signalA->attach(bus1.get());
        signalB->attach(bike1.get());

        // Register them in the traffic control system
        trafficSystem.registerVehicle(car1);
        trafficSystem.registerVehicle(bus1);
        trafficSystem.registerVehicle(bike1);

        // --------------------------------------------------------------------
        // 4. Read road configuration (File Handling)
        // --------------------------------------------------------------------
        // Make sure you have "roads.txt" with lines like:
        // A B 5.0
        // B C 4.0
        // A C 10.0
        std::vector<Road> roads = readRoadsFromFile("roads.txt");

        // Build adjacency list for route optimization
        std::map<std::string, std::vector<std::pair<std::string, double>>> graph;
        for (auto &rd : roads)
        {
            graph[rd.from].push_back({rd.to, rd.distance});
            graph[rd.to].push_back({rd.from, rd.distance}); // assume bidirectional
        }

        // --------------------------------------------------------------------
        // 5. Find shortest path (Algorithms & Data Structures)
        // --------------------------------------------------------------------
        // Let's find a path from A to C
        auto path = findShortestPath(graph, "A", "C");

        // --------------------------------------------------------------------
        // 6. Write path to an output file
        // --------------------------------------------------------------------
        writePathToFile("shortest_path.txt", path);

        // --------------------------------------------------------------------
        // 7. Start Concurrency (Threads)
        // --------------------------------------------------------------------
        bool stopFlag = false;
        std::mutex mtx;
        std::condition_variable cv;

        // Start signal controller threads
        std::thread signalThreadA(signalController, signalA, std::ref(stopFlag), std::ref(mtx), std::ref(cv));
        std::thread signalThreadB(signalController, signalB, std::ref(stopFlag), std::ref(mtx), std::ref(cv));
        std::thread signalThreadC(signalController, signalC, std::ref(stopFlag), std::ref(mtx), std::ref(cv));

        // Start simulation thread
        std::thread simThread(simulationThread, std::ref(stopFlag), std::ref(mtx), std::ref(cv));

        // --------------------------------------------------------------------
        // 8. Let simulation run for a bit, then stop
        // --------------------------------------------------------------------
        std::this_thread::sleep_for(std::chrono::seconds(20));

        {
            std::lock_guard<std::mutex> lock(mtx);
            stopFlag = true;
        }
        cv.notify_all();

        // Wait for threads to finish
        if (signalThreadA.joinable())
            signalThreadA.join();
        if (signalThreadB.joinable())
            signalThreadB.join();
        if (signalThreadC.joinable())
            signalThreadC.join();
        if (simThread.joinable())
            simThread.join();

        // --------------------------------------------------------------------
        // 9. Done
        // --------------------------------------------------------------------
        std::cout << "Simulation complete. Shortest path was written to 'shortest_path.txt'.\n";
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Exception occurred: " << ex.what() << "\n";
    }
    catch (...)
    {
        std::cerr << "Unknown exception occurred.\n";
    }

    return 0;
}
