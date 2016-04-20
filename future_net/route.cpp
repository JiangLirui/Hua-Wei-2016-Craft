#include "route.h"
#include "lib_record.h"
#include "lib_io.h"
#include <stdio.h>

#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <iomanip>

#define EDGE_NUM_MAX 40000
#define NODE_NUM_MAX 2000
#define MUSTNODE_NUM_MAX 100
#define ROUTE_NUM 2
#define MID_RESULT_MAXCOUNT 100

using namespace std;
using std::vector;

char *resultFile;

int edge_info[EDGE_NUM_MAX][4];
int source=-1;
int destination=-1;
int demand_info[2][MUSTNODE_NUM_MAX];
int mustPassCount[2]={0,0};
bool mustPass[ROUTE_NUM][NODE_NUM_MAX];
vector<int> node_position[NODE_NUM_MAX];

int route_num;
vector<int> MID_RESULT_EDGE[ROUTE_NUM][MID_RESULT_MAXCOUNT];
int MID_RESULT_COST[ROUTE_NUM][MID_RESULT_MAXCOUNT];

int result_overlapEdgeCount=INT_MAX;
int result_cost=INT_MAX;
vector<int> result_edge[ROUTE_NUM];

//Dijkstra
vector<int> shortestNode[ROUTE_NUM][MUSTNODE_NUM_MAX+1][MUSTNODE_NUM_MAX+1];
vector<int> shortestEdge[ROUTE_NUM][MUSTNODE_NUM_MAX+1][MUSTNODE_NUM_MAX+1];
int shortestCost[ROUTE_NUM][MUSTNODE_NUM_MAX+1][MUSTNODE_NUM_MAX+1];
int Index[ROUTE_NUM][NODE_NUM_MAX];
void dijkstraBate2(int startNode);

//overlap
vector<int> mustEdge[ROUTE_NUM][MUSTNODE_NUM_MAX+1][2];
struct coordinate
{
    int x;
    int y;
};
vector<coordinate> overlap[ROUTE_NUM][MUSTNODE_NUM_MAX+1][MUSTNODE_NUM_MAX+1];
void test_overlap();

//Test whether Hami is a ring after adding link. if yes, return false;else return true
vector<coordinate> Hami;
bool test_ring(coordinate link);

//IOFirst
bool hasOverlopFlag[MUSTNODE_NUM_MAX+1][MUSTNODE_NUM_MAX+1];
int hasOverlopCount_LC[MUSTNODE_NUM_MAX+1][2];
int passedFlagLC[MUSTNODE_NUM_MAX+1][2];
int mid_result_count=0;
int Count_MAX=0;
bool IoDegreeFirst(int cost,int PassedCount);

bool test_result(vector<int> edge);

int getCount_overlapEdge(vector<int> edgeA, vector<int> edgeB);

void search_route(char* topo[40000], int edge_num, char* demand[2],char *result_file)
{
    //get data
    resultFile=result_file;
    const char* split=",\n\\|";
    char* token;
    for(int i=0;i<edge_num;i++)
    {
        token=strtok(*(topo+i),split);
        for(int j=0;token;j++)
        {
            edge_info[i][j]=atoi(token);
            token=strtok(NULL,split);
        }
    }
    for(int i=0;i<ROUTE_NUM;i++)
    {
    	token=strtok(*(demand+i),split);
    	if(i==0)
    	{
    		token=strtok(NULL,split);
    		source=atoi(token);
    		token=strtok(NULL,split);
    		destination=atoi(token);
    	}
    	else
    	{
    		if(atoi(strtok(NULL,split))!=source)
    		{
    			return;
    		}
    		if(atoi(strtok(NULL,split))!=destination)
    		{
    			return;
    		}
    	}
    	
    	token=strtok(NULL,split);
    	for(int j=0;token;j++)
    	{
        	demand_info[i][j]=atoi(token);
        	mustPassCount[i]++;
        	token=strtok(NULL,split);
    	}
    }

     //data position, and Sort
    for(int i=0;i<edge_num;i++)
    {
        int j=edge_info[i][1];
        if(j!=destination)
        {
            node_position[j].push_back(i);
            if(node_position[j].size()>1)
            {
                for(unsigned k=node_position[j].size()-1;k>0;k--)
                {
                    if(edge_info[node_position[j][k]][3]<edge_info[node_position[j][k-1]][3])
                    {
                        int temp=node_position[j][k];
                        node_position[j][k]=node_position[j][k-1];
                        node_position[j][k-1]=temp;
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
    }
    /*for(int i=0;i<NODE_NUM_MAX;i++)
    {
        if(!node_position[i].empty())
        {
            cout<<i<<":";
            for(unsigned j=0;j<node_position[i].size();j++)
            {
                cout<<" "<<node_position[i][j];
            }
            cout<<endl;
        }
    }*/

    //DFS data Init
    int NodeCount;
    NodeCount=0;
    for(int i=0;i<NODE_NUM_MAX;i++)
    {
        if(!node_position[i].empty())
        {
            	NodeCount++;
       	}
    }

    //Dijkstra
    for(int i=0;i<ROUTE_NUM;i++)
    {
    	route_num=i;

    	//Dijkstra Init
    	for(int j=0;j<NODE_NUM_MAX;j++)
    	{
        	mustPass[i][j]=false;
    	}
    	for(int j=0;j<mustPassCount[i];j++)
    	{
        	mustPass[i][demand_info[i][j]]=true;
    	}

    	for(int j=0;j<MUSTNODE_NUM_MAX+1;j++)
    	{
        	for(int k=0;k<MUSTNODE_NUM_MAX+1;k++)
        	{
            		shortestCost[i][j][k]=INT_MAX;
        	}
    	}

    	for(int j=0;j<mustPassCount[i];j++)
    	{
            	Index[i][demand_info[i][j]]=j;
    	}
    	Index[i][source]=mustPassCount[i];
    	Index[i][destination]=mustPassCount[i];

    	//run dijkstra
    	dijkstraBate2(source);
    	for(int j=0;j<mustPassCount[i];j++)
    	{
    		dijkstraBate2(demand_info[i][j]);
    	}

    	/*for(int j=0;j<mustPassCount[i]+1;j++)
    	{
    		for(int k=0;k<mustPassCount[i]+1;k++)
    		{
    			if(shortestCost[i][j][k]<INT_MAX)
            		{
                		cout<<setw(4)<<shortestCost[i][j][k];
            		}
            		else
            		{
                		cout<<setw(4)<<" $";
            		}
    		}
    		cout<<endl;
    	}
    	cout<<endl;*/
    	//L and C cost!=INT_MAX number,  and SORT
    	for(int j=0;j<mustPassCount[i]+1;j++)
    	{
        	for(int k=0;k<mustPassCount[i]+1;k++)
        	{
            		if(shortestCost[i][j][k]<INT_MAX)
            		{
                		mustEdge[i][j][0].push_back(k);
                		for(unsigned p=mustEdge[i][j][0].size()-1;p>0;p--)
                		{
                    			if(shortestCost[i][j][k]<shortestCost[i][j][mustEdge[i][j][0][p-1]])
                    			{
                        			mustEdge[i][j][0][p]=mustEdge[i][j][0][p-1];
                        			mustEdge[i][j][0][p-1]=k;
                    			}
                    			else
                    			{
                       				break;
                    			}
               	 		}
            		}
            		if(shortestCost[i][k][j]<INT_MAX)
            		{
                		mustEdge[i][j][1].push_back(k);
                		for(unsigned p=mustEdge[i][j][1].size()-1;p>0;p--)
                		{
                    			if(shortestCost[i][k][j]<shortestCost[i][mustEdge[i][j][1][p-1]][j])
                    			{
                        			mustEdge[i][j][1][p]=mustEdge[i][j][1][p-1];
                        			mustEdge[i][j][1][p-1]=k;
                    			}
                    			else
                    			{
                       				break;
                    			}
                		}
            		}
        	}
    	}

    	//OUT=0 &&IN=0
    	for(int j=0;j<mustPassCount[i]+1;j++)
    	{
        	if(mustEdge[i][j][0].empty() || mustEdge[i][j][1].empty())
        	{
            		write_result(resultFile);
            		return;
        	}
    	}

    	test_overlap();
    	/*for(int j=0;j<mustPassCount[i]+1;j++)
    	{
        	for(int k=0;k<mustPassCount[i]+1;k++)
        	{
            		if(!overlap[i][j][k].empty())
            		{
                		cout<<"("<<j<<","<<k<<"):";
                		for(unsigned p=0;p<overlap[i][j][k].size();p++)
                		{
                    		cout<<"("<<overlap[i][j][k][p].x<<","<<overlap[i][j][k][p].y<<") ";
                		}
                		cout<<endl;
            		}
        	}
    	}
    	cout<<endl;*/
    }
    for(int i=0;i<ROUTE_NUM;i++)
    {
    	//route_num=i;
    	if(mustPassCount[0]<=mustPassCount[1])
    	{
    		route_num=i;
    	}
    	else
    	{
    		route_num=ROUTE_NUM-1-i;
    	}

    	//Init
    	Count_MAX=mustPassCount[route_num];
    	for(int j=0;j<mustPassCount[route_num]+1;j++)
    	{
        	hasOverlopCount_LC[j][0]=0;
        	hasOverlopCount_LC[j][1]=0;
    	}
    	for(int j=0;j<mustPassCount[route_num]+1;j++)
    	{
        	for(int k=0;k<mustPassCount[route_num]+1;k++)
        	{
            		if(shortestCost[route_num][j][k]<INT_MAX)
            		{
               		 	hasOverlopFlag[j][k]=false;
            		}
            		else
           	 	{
                		hasOverlopFlag[j][k]=true;
                		hasOverlopCount_LC[j][0]++;
                		hasOverlopCount_LC[k][1]++;
            		}
        	}
    	}
    	for(int j=0;j<mustPassCount[route_num]+1;j++)
    	{
        	passedFlagLC[j][0]=-1;
        	passedFlagLC[j][1]=-1;
    	}

    	if(mustPassCount[route_num]<40)
    	{
        	Count_MAX=5-mustPassCount[route_num]/10;
    	}
    	else
    	{
        	Count_MAX=2;
    	}

    	IoDegreeFirst(0,0);
    	result_cost=INT_MAX;
    	mid_result_count=0;

    	cout<<endl<<"----------------------------------------------------------------------"<<endl;
    }

    for(int i=MID_RESULT_MAXCOUNT-1;i>=0;i--)
    {
    	if(!MID_RESULT_EDGE[0][i].empty())
    	{
    		for(int j=MID_RESULT_MAXCOUNT-1;j>=0;j--)
    		{
    			if(!MID_RESULT_EDGE[1][j].empty())
    			{
    				int overlapCount=getCount_overlapEdge(MID_RESULT_EDGE[0][i],MID_RESULT_EDGE[1][j]);
    				if(overlapCount<result_overlapEdgeCount)
    				{
    					result_overlapEdgeCount=overlapCount;
    					result_cost=MID_RESULT_COST[0][i]+MID_RESULT_COST[1][j];
    					result_edge[0]=MID_RESULT_EDGE[0][i];
    					result_edge[1]=MID_RESULT_EDGE[1][j];

    					cout<<"Batter route pair ("<<i<<","<<j<<")"<<endl;
    					cout<<"overlapCount:"<<result_overlapEdgeCount<<endl;
    					cout<<"route0 cost:"<<MID_RESULT_COST[0][i]<<endl;
    					cout<<"route1 cost:"<<MID_RESULT_COST[1][j]<<endl;
    					cout<<"sum cost:"<<result_cost<<endl<<endl;
    				}
    				else if(overlapCount==result_overlapEdgeCount)
    				{
    					if(MID_RESULT_COST[0][i]+MID_RESULT_COST[1][j]<result_cost)
    					{
    						result_cost=MID_RESULT_COST[0][i]+MID_RESULT_COST[1][j];
    						result_edge[0]=MID_RESULT_EDGE[0][i];
    						result_edge[1]=MID_RESULT_EDGE[1][j];

    						cout<<"Batter route pair ("<<i<<","<<j<<")"<<endl;
    						cout<<"overlapCount:"<<result_overlapEdgeCount<<endl;
    						cout<<"route0 cost:"<<MID_RESULT_COST[0][i]<<endl;
    						cout<<"route1 cost:"<<MID_RESULT_COST[1][j]<<endl;
    						cout<<"sum cost:"<<result_cost<<endl<<endl;
    					}
    				}
    			}
    		}
    	}
    }

    /*for(int i=0;i<ROUTE_NUM;i++)
    {
    	cout<<result_edge[i][0];
    	for(unsigned j=1;j<result_edge[i].size();j++)
    	{
    		cout<<"|"<<result_edge[i][j];
    	}
    	cout<<endl;
    }*/

    if(result_cost==INT_MAX)
    {
        write_result(resultFile);
    }
}

bool IoDegreeFirst(int cost,int PassedCount)
{
    bool BACK=false;

    coordinate nextLink;
    nextLink.x=-1;
    nextLink.y=-1;
    if(PassedCount==mustPassCount[route_num])
    {
        for(int i=0;i<mustPassCount[route_num]+1;i++)
        {
            if(passedFlagLC[i][0]<0)
            {
                nextLink.x=i;
            }
            if(passedFlagLC[i][1]<0)
            {
                nextLink.y=i;
            }
        }

        if(!hasOverlopFlag[nextLink.x][nextLink.y] && !test_ring(nextLink))
        {
            //if(cost+shortestCost[route_num][nextLink.x][nextLink.y]<result_cost)
            //{
                passedFlagLC[nextLink.x][0]=nextLink.y;
                passedFlagLC[nextLink.y][1]=nextLink.x;

                vector<int> ROUTE;
                ROUTE.push_back(mustPassCount[route_num]);
                int nextstart=mustPassCount[route_num];
                do
                {
                    nextstart=passedFlagLC[nextstart][0];
                    ROUTE.push_back(nextstart);
                }while(nextstart!=mustPassCount[route_num]);

                vector<int> futureEdge;
                for(unsigned j=0;j<ROUTE.size()-1;j++)
                {
                    int temp_L=ROUTE[j];
                    int temp_C=ROUTE[j+1];
                    for(unsigned k=0;k<shortestEdge[route_num][temp_L][temp_C].size();k++)
                    {
                        futureEdge.push_back(shortestEdge[route_num][temp_L][temp_C][k]);
                    }
                }
                if(test_result(futureEdge))
                {
                    result_cost=cost+shortestCost[route_num][nextLink.x][nextLink.y];
                    cout<<"Better Route "<<route_num<<":"<<result_cost<<endl;
                    MID_RESULT_COST[route_num][mid_result_count]=result_cost;
                    MID_RESULT_EDGE[route_num][mid_result_count]=futureEdge;
                    mid_result_count++;
                    if(mid_result_count==MID_RESULT_MAXCOUNT)
                    {
                    	BACK=true;
                    }
                }

                passedFlagLC[nextLink.y][1]=-1;
                passedFlagLC[nextLink.x][0]=-1;
            //}
        }
    }
    else
    {
        int L=-1;
        int C=-1;
        for(int i=0,maxSize=0;i<mustPassCount[route_num]+1;i++)
        {
            if(passedFlagLC[i][0]<0)
            {
                if(hasOverlopCount_LC[i][0]>maxSize)
                {
                    maxSize=hasOverlopCount_LC[i][0];
                    L=i;
                    C=0;
                }
                else if(hasOverlopCount_LC[i][0]==maxSize)
                {
                    int cost1=0;
                    int cost2=0;
                    for(unsigned j=0;j<mustEdge[route_num][i][0].size();j++)
                    {
                        if(!hasOverlopFlag[i][mustEdge[route_num][i][0][j]])
                        {
                            cost1=shortestCost[route_num][i][mustEdge[route_num][i][0][j]];
                            break;
                        }
                    }
                    for(unsigned j=0;j<mustEdge[route_num][L][C].size();j++)
                    {
                        if(!hasOverlopFlag[L][mustEdge[route_num][L][C][j]])
                        {
                            cost2=shortestCost[route_num][L][mustEdge[route_num][L][C][j]];
                            break;
                        }
                    }
                    if(cost1<cost2)
                    {
                        /*if(cost+cost1>=result_cost)
                        {
                            return false;
                        }*/
                        L=i;
                        C=0;
                    }
                    else
                    {
                        /*if(cost+cost2>=result_cost)
                        {
                            return false;
                        }*/
                    }
                } 
            }
            if(passedFlagLC[i][1]<0)
            {
                if(hasOverlopCount_LC[i][1]>maxSize)
                {
                    maxSize=hasOverlopCount_LC[i][1];
                    L=i;
                    C=1;
                }
                else if(hasOverlopCount_LC[i][1]==maxSize)
                {
                    int cost1=0;
                    int cost2=0;
                    for(unsigned j=0;j<mustEdge[route_num][i][1].size();j++)
                    {
                        if(!hasOverlopFlag[i][mustEdge[route_num][i][1][j]])
                        {
                            cost1=shortestCost[route_num][i][mustEdge[route_num][i][1][j]];
                            break;
                        }
                    }
                    for(unsigned j=0;j<mustEdge[route_num][L][C].size();j++)
                    {
                        if(!hasOverlopFlag[L][mustEdge[route_num][L][C][j]])
                        {
                            cost2=shortestCost[route_num][L][mustEdge[route_num][L][C][j]];
                            break;
                        }
                    }
                    if(cost1<cost2)
                    {
                        /*if(cost+cost1>=result_cost)
                        {
                            return false;
                        }*/
                        L=i;
                        C=1;
                    }
                    else
                    {
                        /*if(cost+cost2>=result_cost)
                        {
                            return false;
                        }*/
                    }
                }
            }
        }

        if(C==0)
        {
            nextLink.x=L;
        }
        else
        {
            nextLink.y=L;
        }

        int Count=0;
        for(unsigned q=0;q<mustEdge[route_num][L][C].size();q++)
        {
            if(C==0)
            {
                nextLink.y=mustEdge[route_num][L][0][q];
            }
            else
            {
                nextLink.x=mustEdge[route_num][L][1][q];
            }

            if(hasOverlopFlag[nextLink.x][nextLink.y])
            {
                continue;
            }

            /*if(cost+shortestCost[route_num][nextLink.x][nextLink.y]>=result_cost)
            {
                break;
            }*/

            Count++;

            if(test_ring(nextLink))
            {
                    bool GO=true;
                    bool changeFlag1[overlap[route_num][nextLink.x][nextLink.y].size()];
                    for(unsigned k=0;k<overlap[route_num][nextLink.x][nextLink.y].size();k++)
                    {
                        changeFlag1[k]=false;
                    }
                    int temp_L,temp_C;
                    for(unsigned k=0;k<overlap[route_num][nextLink.x][nextLink.y].size();k++)
                    {
                        temp_L=overlap[route_num][nextLink.x][nextLink.y][k].x;
                        temp_C=overlap[route_num][nextLink.x][nextLink.y][k].y;
                        if(!hasOverlopFlag[temp_L][temp_C])
                        {
                            if(hasOverlopCount_LC[temp_L][0]==mustPassCount[route_num] || hasOverlopCount_LC[temp_C][1]==mustPassCount[route_num])
                            {
                                GO=false;
                                break;
                            }
                            else
                            {
                                hasOverlopFlag[temp_L][temp_C]=true;
                                changeFlag1[k]=true;
                                hasOverlopCount_LC[temp_L][0]++;
                                hasOverlopCount_LC[temp_C][1]++;
                            }
                        }
                    }
                    if(GO)
                    {
                        bool changeFlag2[mustPassCount[route_num]+1][2];
                        for(int k=0;k<mustPassCount[route_num]+1;k++)
                        {
                            changeFlag2[k][0]=false;
                            changeFlag2[k][1]=false;
                        }
                        for(int k=0;k<mustPassCount[route_num]+1;k++)
                        {
                            if(!hasOverlopFlag[nextLink.x][k] && k!=nextLink.y)
                            {
                                if(hasOverlopCount_LC[k][1]==mustPassCount[route_num])
                                {
                                    GO=false;
                                    break;
                                }
                                else
                                {
                                    hasOverlopFlag[nextLink.x][k]=true;
                                    changeFlag2[k][0]=true;
                                    hasOverlopCount_LC[k][1]++;
                                }
                            }
                            if(!hasOverlopFlag[k][nextLink.y] && k!=nextLink.x)
                            {
                                if(hasOverlopCount_LC[k][0]==mustPassCount[route_num])
                                {
                                    GO=false;
                                    break;
                                }
                                else
                                {
                                    hasOverlopFlag[k][nextLink.y]=true;
                                    changeFlag2[k][1]=true;
                                    hasOverlopCount_LC[k][0]++;
                                }
                            }
                        }
                        if(GO)
                        {
                            passedFlagLC[nextLink.x][0]=nextLink.y;
                            passedFlagLC[nextLink.y][1]=nextLink.x;
                            BACK=IoDegreeFirst(cost+shortestCost[route_num][nextLink.x][nextLink.y],PassedCount+1);
                            passedFlagLC[nextLink.y][1]=-1;
                            passedFlagLC[nextLink.x][0]=-1;
                        }
                        for(int k=0;k<mustPassCount[route_num]+1;k++)
                        {
                            if(changeFlag2[k][0])
                            {
                                hasOverlopFlag[nextLink.x][k]=false;
                                hasOverlopCount_LC[k][1]--;
                            }
                            if(changeFlag2[k][1])
                            {
                                hasOverlopFlag[k][nextLink.y]=false;
                                hasOverlopCount_LC[k][0]--;
                            }
                        }
                    }
                    for(unsigned k=0;k<overlap[route_num][nextLink.x][nextLink.y].size();k++)
                    {
                        if(changeFlag1[k])
                        {
                            temp_L=overlap[route_num][nextLink.x][nextLink.y][k].x;
                            temp_C=overlap[route_num][nextLink.x][nextLink.y][k].y;

                            hasOverlopFlag[temp_L][temp_C]=false;
                            hasOverlopCount_LC[temp_L][0]--;
                            hasOverlopCount_LC[temp_C][1]--;
                        }
                    }
            }
            if(BACK)
            {
            	break;
            }
            if(Count==Count_MAX)
            {
                break;
            }
        }
    }

    return BACK;
}

void dijkstraBate2(int startNode)
{
    vector<int> shortest_node[NODE_NUM_MAX];
    vector<int> shortest_edge[NODE_NUM_MAX];
    int shortest_cost[NODE_NUM_MAX];
    bool passed[NODE_NUM_MAX];
    int passedMustCount=0;
    int Line=Index[route_num][startNode];
    int Column;

    for(int i=0;i<NODE_NUM_MAX;i++)
    {
        shortest_cost[i]=INT_MAX;
        passed[i]=false;
    }

    if(startNode==source)
    {
        passed[destination]=true;
    }
    else if(mustPass[route_num][startNode])
    {
        passed[source]=true;
        passedMustCount++;
    }

    int min_cost;
    int nextNode=startNode;
    shortest_cost[nextNode]=0;
    passed[nextNode]=true;
    do
    {
        if(!node_position[nextNode].empty()/* && (!mustPass[route_num][nextNode] || nextNode==startNode)*/)
        {
            for(unsigned short i=0;i<node_position[nextNode].size();i++)
            {
                int j=node_position[nextNode][i];
                int out=edge_info[j][2];
                if(!passed[out] && shortest_cost[nextNode]+edge_info[j][3]<shortest_cost[out])
                {
                    shortest_cost[out]=shortest_cost[nextNode]+edge_info[j][3];
                    shortest_node[out]=shortest_node[nextNode];
                    shortest_node[out].push_back(out);
                    shortest_edge[out]=shortest_edge[nextNode];
                    shortest_edge[out].push_back(edge_info[j][0]);
                }
            }
        }

        min_cost=INT_MAX;
        for(int i=0;i<NODE_NUM_MAX;i++)
        {
            if(!passed[i] && shortest_cost[i]<min_cost)
            {
                min_cost=shortest_cost[i];
                nextNode=i;
            }
        }

        if(!passed[nextNode])
        {
            if(startNode==source)
            {
                if(mustPass[route_num][nextNode])
                {
                    bool Repect=false;
                    for(unsigned i=0;i<shortest_node[nextNode].size()-1;i++)
                    {
                        if(mustPass[route_num][shortest_node[nextNode][i]])
                        {
                            Repect=true;
                            break;
                        }
                    }
                    if(!Repect)
                    {
                        Column=Index[route_num][nextNode];
                        shortestCost[route_num][Line][Column]=shortest_cost[nextNode];
                        shortestNode[route_num][Line][Column]=shortest_node[nextNode];
                        shortestEdge[route_num][Line][Column]=shortest_edge[nextNode];
                   }

                    passedMustCount++;
                    if(passedMustCount==mustPassCount[route_num])
                    {
                        break;
                    }
                }
            }
            else if(mustPass[route_num][startNode])
            {
                if(mustPass[route_num][nextNode] || nextNode==destination)
                {
                    bool Repect=false;
                    for(unsigned i=0;i<shortest_node[nextNode].size()-1;i++)
                    {
                        if(mustPass[route_num][shortest_node[nextNode][i]])
                        {
                            Repect=true;
                            break;
                        }
                    }
                    if(!Repect)
                    {
                        Column=Index[route_num][nextNode];
                        shortestCost[route_num][Line][Column]=shortest_cost[nextNode];
                        shortestNode[route_num][Line][Column]=shortest_node[nextNode];
                        shortestEdge[route_num][Line][Column]=shortest_edge[nextNode];
                    }

                    passedMustCount++;
                    if(passedMustCount==mustPassCount[route_num]+1)
                    {
                        break;
                    }
                }
            }
        }

        passed[nextNode]=true;
    }while(min_cost!=INT_MAX);
}

void test_overlap()
{
    bool node_flag[NODE_NUM_MAX];
    for(int i=0;i<mustPassCount[route_num]+1;i++)
    {
        for(int j=0;j<mustPassCount[route_num]+1;j++)
        {
            if(shortestCost[route_num][i][j]<INT_MAX)
            {
                for(int k=0;k<NODE_NUM_MAX;k++)
                {
                    node_flag[k]=false;
                }
                for(unsigned k=0;k<shortestNode[route_num][i][j].size();k++)
                {
                    node_flag[shortestNode[route_num][i][j][k]]=true;
                }
                for(int p=i+1;p<mustPassCount[route_num]+1;p++)
                {
                    for(int q=0;q<mustPassCount[route_num]+1;q++)
                    {
                        if(q!=j && shortestCost[route_num][p][q]<INT_MAX)
                        {
                            for(unsigned k=0;k<shortestNode[route_num][p][q].size();k++)
                            {
                                if(node_flag[shortestNode[route_num][p][q][k]])
                                {
                                    coordinate temp1={p,q};
                                    coordinate temp2={i,j};

                                    overlap[route_num][i][j].push_back(temp1);
                                    overlap[route_num][p][q].push_back(temp2);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

bool test_ring(coordinate link)
{
    int nextstart=link.y;
    while(passedFlagLC[nextstart][0]>=0)
    {
        if(passedFlagLC[nextstart][0]==link.x)
        {
            return false;
        }
        else
        {
            nextstart=passedFlagLC[nextstart][0];
        }
    }
    return true;
}

bool test_result(vector<int> edge)
{
    bool Node[NODE_NUM_MAX];
    bool Edge[EDGE_NUM_MAX];
    int midCount=0;
    for(int i=0;i<NODE_NUM_MAX;i++)
    {
        Node[i]=false;
    }
    for(int i=0;i<EDGE_NUM_MAX;i++)
    {
        Edge[i]=false;
    }
    int end=source;
    Node[end]=true;
    for(unsigned i=0;i<edge.size();i++)
    {
        int j=edge[i];
        if(Edge[j])
        {
            cout<<"Repress link!"<<endl;
            return false;
        }
        if(edge_info[j][1]==end)
        {
            if(i==(edge.size()-1))
            {
                if(edge_info[j][2]!=destination)
                {
                    cout<<"last node errer!"<<endl;
                    return false;
                }
            }
            if(Node[edge_info[j][2]])
            {
                cout<<"Repress Node! "<<endl;
                return false;
            }
            else
            {
                end=edge_info[j][2];
                if(mustPass[route_num][end])
                {
                    midCount++;
                }
            }
        }
        else
        {
            if(i==0)
            {
                cout<<"first node errer!"<<endl;
            }
            else
            {
                cout<<"It isn't a series!"<<endl;
            }
            return false;
        }
    }

    if(midCount!=mustPassCount[route_num])
    {
        cout<<"Unreach node!"<<endl;
        return false;
    }
    return true;
}

int getCount_overlapEdge(vector<int> edgeA, vector<int> edgeB)
{
	int Count=0;
	for(unsigned i=0;i<edgeA.size();i++)
	{
		for(unsigned j=0;j<edgeB.size();j++)
		{
			if(edgeA[i]==edgeB[j])
			{
				Count++;
				break;
			}
		}
	}
	return Count;
}
