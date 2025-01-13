from typing import Optional, List, Dict

class GpsInfo:
    def __init__(self, lat: float, long: float, alt: float):
        self.lat = lat
        self.long = long
        self.alt = alt

class UtmInfo:
    def __init__(self, easting: float, northing: float, zone: str):
        self.easting = easting
        self.northing = northing
        self.zone = zone

class Node:
    def __init__(
        self,
        id: str,
        admin_code: Optional[str],
        node_type: int,
        its_node_id: Optional[str],
        maker: str,
        update_date: str,
        version: str,
        remark: str,
        hist_type: str,
        hist_remark: str,
        gps_info: Dict[str, float],
        utm_info: Dict[str, float],
    ):
        self.id = id
        self.admin_code = admin_code
        self.node_type = node_type
        self.its_node_id = its_node_id
        self.maker = maker
        self.update_date = update_date
        self.version = version
        self.remark = remark
        self.hist_type = hist_type
        self.hist_remark = hist_remark
        self.gps_info = GpsInfo(**gps_info)
        self.utm_info = UtmInfo(**utm_info)

    @staticmethod
    def from_dict(data: Dict):
        return Node(
            id=data["ID"],
            admin_code=data["AdminCode"],
            node_type=data["NodeType"],
            its_node_id=data["ITSNodeID"],
            maker=data["Maker"],
            update_date=data["UpdateDate"],
            version=data["Version"],
            remark=data["Remark"],
            hist_type=data["HistType"],
            hist_remark=data["HistRemark"],
            gps_info=data["GpsInfo"],
            utm_info=data["UtmInfo"],
        )

class Link:
    def __init__(
        self,
        id: str,
        admin_code: str,
        road_rank: int,
        road_type: int,
        road_no: str,
        link_type: int,
        lane_no: int,
        r_link_id: str,
        l_link_id: str,
        from_node_id: str,
        to_node_id: str,
        section_id: str,
        length: float,
        its_link_id: str,
        maker: str,
        update_date: str,
        version: str,
        remark: str,
        hist_type: str,
        hist_remark: str,
    ):
        self.id = id
        self.admin_code = admin_code
        self.road_rank = road_rank
        self.road_type = road_type
        self.road_no = road_no
        self.link_type = link_type
        self.lane_no = lane_no
        self.r_link_id = r_link_id
        self.l_link_id = l_link_id
        self.from_node_id = from_node_id
        self.to_node_id = to_node_id
        self.section_id = section_id
        self.length = length
        self.its_link_id = its_link_id
        self.maker = maker
        self.update_date = update_date
        self.version = version
        self.remark = remark
        self.hist_type = hist_type
        self.hist_remark = hist_remark

    @staticmethod
    def from_dict(data: Dict):
        return Link(
            id=data["ID"],
            admin_code=data["AdminCode"],
            road_rank=data["RoadRank"],
            road_type=data["RoadType"],
            road_no=data["RoadNo"],
            link_type=data["LinkType"],
            lane_no=data["LaneNo"],
            r_link_id=data["R_LinkID"],
            l_link_id=data["L_LinkID"],
            from_node_id=data["FromNodeID"],
            to_node_id=data["ToNodeID"],
            section_id=data["SectionID"],
            length=data["Length"],
            its_link_id=data["ITSLinkID"],
            maker=data["Maker"],
            update_date=data["UpdateDate"],
            version=data["Version"],
            remark=data["Remark"],
            hist_type=data["HistType"],
            hist_remark=data["HistRemark"],
        )