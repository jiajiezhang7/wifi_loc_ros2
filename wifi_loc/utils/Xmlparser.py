import sys
import xml.etree.ElementTree as ET
import pickle
from collections import Counter
from .read_pickle import RssData, read_data_from_pickle

class OsmDataParser:
    def __init__(self, osm_file):
        self.osm_file = osm_file
        self.nodes = {}
        self.target_nodes = {}
        self.rssi_value = {}
        self.rssi = []
        self.ap_to_position = {}
        self.ap_level = {}
        self.target_ap = {}
        self.way_data = []
        self.tree = ET.parse(self.osm_file)
        self.root = self.tree.getroot()
        self.all_mac = []

    def parse(self):
        for element in self.root:
            if element.tag == 'node':
                self._parse_node(element)
            elif element.tag == 'way':
                self._parse_way(element)

    def _parse_node(self, element):
        node_id = element.attrib['id']
        lat = float(element.attrib['lat'])
        lon = float(element.attrib['lon'])

        self.nodes[node_id] = (lon, lat)

        tags = list(element.iter('tag'))
        for i, tag in enumerate(tags):
            if 'osmAG:WiFi:BSSID:5G' in tag.attrib.get('k'):
                self._parse_wifi_node(tag, tags, lon, lat)
            if tag.attrib.get('v') == 'RP':
                self._parse_rp_node(tags, lon, lat, tag.attrib.get('k'))

    def _parse_wifi_node(self, tag, tags, lon, lat):
        self.ap_to_position[tag.attrib.get('v').lower()] = (lon, lat)
        for tag_ in tags:
            if tag_.attrib.get('k') == 'osmAG:WiFi:level':
                level_tag = tag_.attrib.get('v')
            
        self.ap_level[tag.attrib.get('v').lower()] = int(level_tag)

    def _parse_rp_node(self, tags, lon, lat, tag_key):
        path_tag = None
        level_tag = None
        for tag_ in tags:
            if tag_.attrib.get('k') == 'path':
                path_tag = tag_.attrib.get('v')
            if tag_.attrib.get('k') == 'level':
                level_tag = tag_.attrib.get('v')

        if path_tag:
            # 使用read_data_from_pickle函数来加载数据
            data = read_data_from_pickle(path_tag)
            
            self.target_ap[(lon, lat)] = self.target_ap.get((lon, lat), {})
            self.target_ap[(lon, lat)]['mac'] = {}
            self.target_ap[(lon, lat)]['level'] = int(level_tag)
            
            for msg in data:
                mac_addresses = msg.mac_address
                rss_val = msg.data

                for i, mac in enumerate(mac_addresses):
                    rss_list = rss_val[i].rss
                   
                    if mac not in self.all_mac:
                        self.all_mac.append(mac)
                        
                    if mac not in self.target_ap[(lon, lat)]['mac']:
                        self.target_ap[(lon, lat)]['mac'][mac] = []
                        self.target_ap[(lon, lat)]['level'] = int(level_tag)
                        self.target_ap[(lon, lat)]['RP'] = tag_key
                    self.target_ap[(lon, lat)]['mac'][mac].extend(rss_list)

    def _parse_way(self, element):
        way_nodes = [self.nodes[nd.attrib['ref']] for nd in element.iter('nd') if nd.attrib['ref'] in self.nodes]
        if way_nodes:
            way_level = None
            for tag in element.iter('tag'):
                if tag.attrib['k'] == 'level':
                    way_level = tag.attrib['v']

            way_tuple = tuple(way_nodes)
            if way_level:
                self.way_data.append((way_tuple, way_level))

    def get_data(self):
        return self.ap_to_position, self.ap_level, self.target_ap, self.way_data, self.all_mac
