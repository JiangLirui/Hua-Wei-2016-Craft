#include "route.h"
#include "lib_io.h"
#include "lib_time.h"
#include "stdio.h"

int main(int argc, char *argv[])
{
    print_time("Begin");
    char *topo[40000];
    int edge_num;
    char *demand[2];
    int demand_num;

    char *topo_file = argv[1];
    edge_num = read_file(topo, 40000, topo_file);
    if (edge_num == 0)
    {
        printf("Please input valid topo file.\n");
        return -1;
    }
    char *demand_file = argv[2];
    demand_num = read_file(demand, 2, demand_file);
    if (demand_num != 2)
    {
        printf("Please input valid demand file.\n");
        return -1;
    }

    char *result_file ;
    result_file = argv[3];

    search_route(topo, edge_num, demand,result_file);

    release_buff(topo, edge_num);
    release_buff(demand, demand_num);

    print_time("End");

	return 0;
}

