module RoutingTableP {
	provides interface RoutingTable;

	uses interface Hashmap<int> as routingTable;
}

implementation {
	command void RoutingTable.update(uint16_t i, uint16_t nextHop) {
		call routingTable.insert(i, nextHop);
	}

	command uint16_t RoutingTable.get(uint16_t i) {
		return call routingTable.get(i);
	}
}
