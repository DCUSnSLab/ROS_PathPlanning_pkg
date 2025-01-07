def calculate_utm_zone(lat, lon):
    """
    Calculate the UTM Zone and Hemisphere based on GPS coordinates.

    Parameters:
        lat (float): Latitude in decimal degrees.
        lon (float): Longitude in decimal degrees.

    Returns:
        dict: A dictionary containing the UTM Zone and Hemisphere.
    """
    # Calculate UTM Zone
    zone = int((lon + 180) // 6) + 1

    # Determine Hemisphere
    hemisphere = "North" if lat >= 0 else "South"

    return {
        "UTM_Zone": f"{zone}{'N' if hemisphere == 'North' else 'S'}",
        "Zone_Number": zone,
        "Hemisphere": hemisphere
    }


# Example usage
lat = 37.23941609958806
lon = 126.77325708203671

utm_info = calculate_utm_zone(lat, lon)
print(utm_info)
