***********************
OSRM Backend Debug Note
***********************

#. Multi-Level Dijkstra algorithm debug note

    .. code-block:: sh
        :caption: breakpoints for osrm-example

        b osrm::engine::RoutingAlgorithms<osrm::engine::routing_algorithms::mld::Algorithm>::DirectShortestPathSearch
        b osrm::engine::routing_algorithms::directShortestPathSearch<osrm::engine::routing_algorithms::mld::Algorithm>
        b osrm::engine::routing_algorithms::mld::search<osrm::engine::routing_algorithms::mld::Algorithm, osrm::engine::PhantomNodes>
        b osrm::engine::routing_algorithms::mld::routingStep<true, osrm::engine::routing_algorithms::mld::Algorithm, osrm::engine::PhantomNodes>
        b osrm::engine::routing_algorithms::mld::relaxOutgoingEdges<true, osrm::engine::routing_algorithms::mld::Algorithm, osrm::engine::PhantomNodes>

#. **osrm-routed** network framework

    osrm-routed uses ``boost::asio`` as building blocks,
    in fact, it is an multi-threaded server.

    Refer here [#bsd_sockets]_ for the comparison between boost network interfaces and BSD socket interfaces.

.. rubric:: Footnotes

.. [#bsd_sockets] https://www.boost.org/doc/libs/1_46_1/doc/html/boost_asio/overview/networking/bsd_sockets.html

