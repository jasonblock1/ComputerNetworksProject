configuration RoutingTableC{
	provides interface RoutingTable;
}

implementation{
	components RoutingTableP;
	RoutingTable = RoutingTableP.RoutingTable;

	components new HashmapC(int, 300) as routingTableC;
    	RoutingTableP.routingTable -> routingTableC;

	
}
