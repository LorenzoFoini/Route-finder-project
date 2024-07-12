/*
    /////////////////////////////////////////////////////////////////////
    /////                PROGRAM OF FOINI LORENZO                  //////
    /////    PROJECT OF ALGORITHM AND DATA STRUCTURES 2022/2023    //////
    /////////////////////////////////////////////////////////////////////
*/

/*
    Data structures used:
    - One to create the root of the T-tree, later used for operations on the BST
    - BST for saving stations (Nodes).
    - Unordered list for saving cars.
    - Queue for planning the route.
    - List for saving and printing the route.

    I use a BST because then I have all operations in O(h) with h tree height.
    In the worst case, i.e. when the tree is completely unbalanced: h = n (Number of stations).
    In the optimal case, i.e. when the tree is completely balanced: h = log(n).

    The struct for the root T has the following field:
    - root -> Root

    Each node of the BST represents a station.
    Each node contains:
    - node.distance -> Station distance from the beginning of the highway
    - node.left -> Left child
    - node.right -> Right child
    - node.dad -> Father of node
    - node.pi -> Pointer to the predecessor in the route planner
    - node.color -> A char to indicate the color of the node. Used in route planning
    - node.listCars -> Pointer to the head of the list of cars at that station
    - node.maxAutonomy -> Integer indicating the maximum autonomy of a car in the station

    ATTENTION: There is no need for a pointer to the root of the BST because I will use T->root

    I use an unordered list of cars so that the entry is O(1).
    Each node in the list will have the following fields:
    - Autonomy
    - Pointer to next car
*/

/*
    In order to be able to plan the route, I try to readjust breadth-first-search (BFS) algorithm.
    Path-planning reasoning:
    - Direct:
        Starting from the starting station, I make an amplitude visit of that node.
        I "create" the arcs when I can reach a station with the car at maximum amplitude from the starting station.
        I then use the BFS of a graph adapted to this specific case, in particular, the first time I meet the
        arrival station then I stop and will have found the best route.
        I then print that route, if it exists, using the pointer to the predecessor.

    - Reverse:
        I use the same reasoning with appropriate modifications.
        I make a BFS of the arrival node and visit the next node if the maximum autonomy of a car of that node allows
        to reach the arrival node. I then reason in a similar manner.
        I then use the BFS of a graph adapted to this specific case, in particular, the first time I encounter the
        departure station then I stop and will have found the best route.
        I then print that path, if it exists, using the pointer to the predecessor.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>


// Define structs for saving the data

// Struct for list of cars in a station
typedef struct car_ {
    int autonomy;
    struct car_ *next_car;
} car_t;

// Struct for BST's nodes
typedef struct station_t {
    int distance, max_car;
    char color; // Use for planning the route
    struct station_t *right;
    struct station_t *left;
    struct station_t *parent;
    struct station_t *pi; // Use for planning the route
    car_t *list_cars;
}station_t;

// Struct for the tree: Contain root
typedef struct bst_t {
    station_t *root;
}bst_t;

// Struct for the list-queue used in the planining-route
typedef struct list_queue_ {
    station_t *s;
    struct list_queue_ *next;
} list_queue_t ;

// Struct for the queue used in the planning route
typedef struct queue_t {
    list_queue_t *head, *tail;
} queue_t;

// Struct used for print the route
typedef struct route_ {
    int distance;
    struct route_ *next;
}route_t;

// DECLARATION OF SUB-PROGRAMMES
// COMMON OPERATION ON BST
station_t* InitialiseStation(int);
bst_t* InitialiseBST();
station_t* MinimumBST(station_t*);
station_t* MaximumBST(station_t*);
station_t* PredecessorBST(station_t*);
station_t* SuccessorBST(station_t*);
void InsertStationBST(bst_t*, station_t*);
station_t* SearchStationBST(bst_t*, int dist);
void TransplantBST(bst_t*, station_t*, station_t*);
void DemolishStationBST(bst_t*, station_t*);

void AddStation(bst_t*, station_t*, int, int const*);
void AddCar(car_t**, int);
bool ScrapCar(car_t**, int);
int CarMaximumAutonomy(car_t *);

// Sub-programmes for planning the route
void BfsDirect(station_t*, station_t*);
void BfsReverse(station_t*, station_t*);

// Sub-programmes for the queue
void Enqueue(queue_t*, station_t*);
station_t* Dequeue(queue_t*);


// MAIN
int main(){
    // Declare local variables
    char command[20];
    int dist_station, num_cars, car_autonomy, start, end;
    int *vector_autonomy;
    station_t *stat;
    bst_t *T;

    // Initialise BST
    T = InitialiseBST();

    // Read all the commands until the end of the file
    while(scanf("%s", command) != EOF)
    {
        if(strcmp(command, "add-station") == 0)
        { // Read command add station
            // Read now the other data
            if(scanf("%d %d", &dist_station, &num_cars));

            // Check if there is already a station at such distance
            stat = SearchStationBST(T, dist_station);

            if(stat == NULL)
            { // A station at such distance doesn't exist
                // Initialise station
                stat = InitialiseStation(dist_station);
                printf("added\n");

                // Create a dinamic array for saving the cars's autonomy
                vector_autonomy = (int *) malloc( 512 * sizeof(int));
                if(vector_autonomy)
                {
                    for (int i = 0; i < num_cars; ++i)
                        if(scanf("%d", vector_autonomy + i));

                    // Call to method for add station in the BST
                    AddStation(T, stat, num_cars, vector_autonomy);

                    // Free memory
                    free(vector_autonomy);
                }
            }else
                printf("not added\n");

        }else if(strcmp(command,"demolish-station") == 0)
        { // Read command demolish station
            // Read the other data
            if(scanf("%d", &dist_station));

            //Cerco la stazione da demolire e poi richiamo demolisci stazione
            stat = SearchStationBST(T, dist_station);

            if(stat != NULL)
            { // Found station
                DemolishStationBST(T, stat);
                printf("demolished\n");
            }else
                printf("not demolished\n");

        }else if(strcmp(command,"add-car") == 0)
        { // Read command add car
            // Read the other data
            if(scanf("%d %d", &dist_station, &car_autonomy));

            // Find the station where to insert the card
            stat = SearchStationBST(T, dist_station);

            if(stat != NULL)
            {
                AddCar(&(stat->list_cars), car_autonomy);
                if(car_autonomy > stat->max_car)
                    stat->max_car = car_autonomy;

                printf("added\n");
            }else
                printf("not added\n");

        }else if(strcmp(command,"scrap-car") == 0)
        { // Read command scrap car
            // Read the other data
            if(scanf("%d %d", &dist_station, &car_autonomy));

            // Find the station woth the given distance, and then remove the car
            stat = SearchStationBST(T, dist_station);
            if(stat != NULL)
            { // Station founded
                if (ScrapCar(&(stat->list_cars), car_autonomy))
                {
                    printf("scrapped\n");
                    
                    // Check if the removed car was the one with the maximum autonomy
                    if(car_autonomy == stat->max_car)
                        stat->max_car = CarMaximumAutonomy(stat->list_cars);

                }else
                    printf("not scrapped\n"); // Car doesn't exist

            }else
                printf("not scrapped\n"); // Station doesn't exist

        }else if(strcmp(command,"plan-route") == 0)
        { // Read command plan route
            // Read the other data
            if(scanf("%d %d", &start, &end));

            station_t *stat_p = SearchStationBST(T, start);
            station_t *stat_a = SearchStationBST(T, end);

            // Call to method plan route based on the comparison between start and end
            if(start == end)
                printf("%d\n", start);
            else if(start < end)
                BfsDirect(stat_p, stat_a);
            else
                BfsReverse(stat_p, stat_a);
        }
    }

    return 0;
}

// SUB-PROGRAMMES
// Function for initiliase the station
station_t* InitialiseStation(int dist)
{
    station_t *n = (station_t *) malloc(sizeof(station_t));
    n->distance = dist;
    n->color = 'w';
    n->max_car = 0;
    n->left = NULL;
    n->right = NULL;
    n->parent = NULL;
    n->pi = NULL;
    n->list_cars = NULL;

    return n;
}

// Function for initialise the BST
bst_t* InitialiseBST()
{
    bst_t *t = (bst_t *) malloc(sizeof(bst_t));
    t->root = NULL;

    return t;
}

// Function which calculate the minimum in a BST
station_t* MinimumBST(station_t *x)
{
    while(x->left != NULL)
        x = x->left;
    return x;
}

// Function which calculate the maximum in a BST
station_t* MaximumBST(station_t *x)
{
    while(x->right != NULL)
        x = x->right;
    return x;
}

// Function which calculate the predecessor of a node in the BST
station_t* PredecessorBST(station_t *stat)
{
    if(stat->left != NULL)
        return MaximumBST(stat->left);

    station_t *y = stat->parent;
    while(y != NULL && stat == y->left)
    {
        stat = y;
        y = y->parent;
    }
    return y;
}

// Function which calculate the successor of a node in the BST
station_t* SuccessorBST(station_t *stat)
{
    if(stat->right != NULL)
        return MinimumBST(stat->right);

    station_t *y = stat->parent;
    while(y != NULL && stat == y->right)
    {
        stat = y;
        y = y->parent;
    }
    return y;
}

// Function for insert a station in the BST
void InsertStationBST(bst_t *T, station_t *z)
{
    station_t *y = NULL;
    station_t *x = T->root;
    while(x != NULL)
    {
        y = x;
        if(z->distance < x->distance)
            x = x->left;
        else
            x = x->right;
    }
    z->parent = y;

    if(y == NULL)
        T->root = z;
    else if(z->distance < y->distance)
        y->left = z;
    else
        y->right = z;
}

// Function which search a station given its distance (in the BST)
// Return the station if exists, otherwise NULL
station_t* SearchStationBST(bst_t *T, int dist)
{
    station_t *stat = T->root;
    while(stat != NULL && dist != stat->distance)
    {
        if (dist < stat->distance)
            stat = stat->left;
        else
            stat = stat->right;
    }

    return stat;
}

// Function for transplan the BST
void TransplantBST(bst_t *T, station_t *u, station_t *v)
{
    if(u->parent == NULL)
        T->root = v;
    else if(u == u->parent->left)
        u->parent->left = v;
    else
        u->parent->right = v;

    if(v != NULL)
        v->parent = u->parent;
}

// Function which delete a station from the BST
void DemolishStationBST(bst_t *T, station_t *z)
{
    if(z->left == NULL)
    {
        TransplantBST(T, z, z->right);
        free(z);
    }
    else if(z->right == NULL)
    {
        TransplantBST(T, z, z->left);
        free(z);
    }
    else
    {
        station_t *y = MinimumBST(z->right);
        if(y->parent != z)
        {
            TransplantBST(T, y, y->right);
            y->right = z->right;
            y->right->parent = y;
        }
        TransplantBST(T, z, y);
        y->left = z->left;
        y->left->parent = y;
        free(z);
    }
}

// Function which create a station, insert it into the BST and add cars
void AddStation(bst_t *T, station_t *stat, int num_cars, int const *vector_autonomy)
{
    // Insert the cars in the list of cars of the station
    for(int i = 0; i < num_cars; i++)
    {
        AddCar(&(stat->list_cars), *(vector_autonomy + i));
        if(*(vector_autonomy+i) > stat->max_car)
            stat->max_car = *(vector_autonomy+i);
    }

    // Insert station in the BST
    InsertStationBST(T, stat);
}

// Function for adding a given car in the list
void AddCar(car_t **list_cars, int car_autonomy)
{
    car_t *tmp = (car_t *) malloc(sizeof(car_t));

    tmp->autonomy = car_autonomy;
    tmp->next_car = *list_cars;
    *list_cars = tmp;
}

// Function for scrapping a given card, so a remove from list
// Return true adn remove the car if the car is present, otherwise false
bool ScrapCar(car_t **list_cars, int car_autonomy)
{
    // Iterate thorugh the list considering the predecessor for moving the pointers
    car_t *prec = NULL;
    car_t *curr = *(list_cars);

    while(curr && curr->autonomy != car_autonomy)
    {
        prec = curr;
        curr = curr->next_car;
    }

    
    // Now check if there is such card, and remove it
    if(curr)
    {
        if(prec)
            prec->next_car = curr->next_car;
        else
            *list_cars = curr->next_car;

        free(curr);
        return true;
    }else
        return false;
}

// Function which, given a list, calculate the maximum autonomy for a car
int CarMaximumAutonomy(car_t *list)
{
    int max = 0;
    for(car_t *tmp = list; tmp; tmp = tmp->next_car)
    {
        if (tmp->autonomy > max)
            max = tmp->autonomy;
    }

    return max;
}


// Function for calculating and printing the route in ascending order of distance of the stations
// Based on BFS code on a graph
void BfsDirect(station_t *s_p, station_t *s_a)
{
    route_t *route = NULL; // Save and print the route
    bool exit = false;
    s_p->color = 'g';

    // Create queue and insert the distance of the starting station
    queue_t *queue = (queue_t *) malloc(sizeof(queue_t));
    queue->head = NULL;
    queue->tail = NULL;
    Enqueue(queue, s_p);

    while(!exit && queue->head != NULL)
    {
        station_t *curr = Dequeue(queue);
        station_t *reach = SuccessorBST(curr);
        while(!exit && curr->max_car >= reach->distance - curr->distance)
        {
            if(reach->color == 'w')
            {
                reach->color = 'g';
                reach->pi = curr;
                Enqueue(queue, reach);
            }

            if(reach->distance == s_a->distance)
                exit = true;
            else
                reach = SuccessorBST(reach);
        }
    }

    // Insert station in an auxiliary list
    station_t *printStation = s_a;
    while(printStation->pi != NULL)
    {
        //Inserisco in testa alla lista
        route_t *tmp = malloc(sizeof(route_t));
        tmp->distance = printStation->distance;
        tmp->next = route;
        route = tmp;

        // Go to predecessor
        printStation = printStation->pi;
    } // ATTENTION: Start station is not inserted

    // Print route
    if(route == NULL)
        printf("no route\n");
    else
    {
        printf("%d ",s_p->distance); // Start
        for(; route->next != NULL; route = route->next)
            printf("%d ", route->distance);
        printf("%d\n",s_a->distance); // End
    }

    // Re-initialise the colors of all stations to white and the pointer to predecessor to NULL
    for(; s_p->distance <= s_a->distance;)
    {
        s_p->color = 'w';
        s_p->pi = NULL;
        s_p = SuccessorBST(s_p);
        if(s_p == NULL)
            break;
    }

    free(queue);
}

// Function for calculating and printing the route in descending order of distance of the stations
// Based on BFS code on a graph
void BfsReverse(station_t *s_p, station_t *s_a)
{
    bool exit = false;
    s_a->color = 'g';

    // Create a queue and insert the distance of the end station
    queue_t *queue = (queue_t *) malloc(sizeof(queue_t));
    queue->head = NULL;
    queue->tail = NULL;
    Enqueue(queue, s_a);

    while(!exit && queue->head != NULL)
    {
        station_t *curr = Dequeue(queue);
        station_t *reach = SuccessorBST(curr);
        while(!exit)
        {
            if(reach->max_car >= reach->distance - curr->distance)
            {
                if(reach->color == 'w')
                {
                    reach->color = 'g';
                    reach->pi = curr;
                    Enqueue(queue, reach);
                }

                if(reach->distance == s_p->distance)
                    exit = true;
            }
            reach = SuccessorBST(reach);
            if(reach == NULL || reach->distance > s_p->distance)
                break;
        }
    }

    // Print the route
    if(s_p->pi == NULL)
        printf("no route\n");
    else
    {
        station_t *printRoute = s_p;
        while(printRoute->pi != NULL)
        {
            printf("%d ",printRoute->distance);
            printRoute = printRoute->pi;
        }
        printf("%d\n",printRoute->distance); // End
    }
    
    // Re-initialise the colors of all station to white and the pointer to predecessor to NULL
    for (; s_p->distance >= s_a->distance;)
    {
        s_p->color = 'w';
        s_p->pi = NULL;
        s_p = PredecessorBST(s_p);
        if(s_p == NULL)
            break;
    }

    free(queue);
}

// Function for insert at the end of a queue
void Enqueue(queue_t *q, station_t *v)
{
    list_queue_t *node = (list_queue_t *) malloc(sizeof(list_queue_t));
    node->s = v;
    node->next = NULL;
    if(q->head == NULL) // Empty queue
    {
        q->head = node;
        q->tail = node;
    }else
    {
        q->tail->next = node;
        q->tail = node;
    }
}

// Function for remove the first element from a queue
// Return the removed station
station_t* Dequeue(queue_t *q)
{
    station_t *tmp = q->head->s;
    list_queue_t *node = q->head;
    q->head = q->head->next;

    free(node);
    return tmp;
}