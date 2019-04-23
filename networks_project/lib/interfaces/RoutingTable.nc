interface RoutingTable {
	command void update(uint16_t i, uint16_t nextHop);

	command uint16_t get(uint16_t i);
}
