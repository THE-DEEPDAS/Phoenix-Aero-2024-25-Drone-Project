from fpdf import FPDF

# Create PDF instance
pdf = FPDF()
pdf.set_auto_page_break(auto=True, margin=15)

# Title function
def add_title(title):
    pdf.add_page()
    pdf.set_font("Arial", size=16, style='B')
    pdf.cell(0, 10, title, ln=True, align='C')
    pdf.ln(10)

# Section function
def add_section(title, content):
    pdf.set_font("Arial", size=14, style='B')
    pdf.cell(0, 10, title, ln=True)
    pdf.set_font("Arial", size=12)
    pdf.multi_cell(0, 10, content)
    pdf.ln(5)

# Add content to the PDF
def add_content(title, intro, principles, applications):
    add_title(title)
    add_section("Introduction", intro)
    add_section("Principles of Operation", principles)
    add_section("Applications", applications)

# Magnetometer content
magnetometer_intro = """
A magnetometer is an instrument used for measuring magnetic forces, especially the Earth's magnetism. 
It is used in various applications such as geophysical surveys, navigation systems, and scientific research.
"""

magnetometer_principles = """
Magnetometers measure the magnetic field intensity in one or more directions. There are several types of magnetometers:
- Scalar Magnetometers: Measure the total strength of the magnetic field.
- Vector Magnetometers: Measure the components of the magnetic field in specific directions.

Common types include:
- Fluxgate Magnetometers: Use a ferromagnetic core with windings to detect the magnetic field.
- Hall Effect Magnetometers: Use the Hall effect in semiconductors.
- Optically Pumped Magnetometers: Use the interaction of light and magnetic fields in certain gases.
"""

magnetometer_applications = """
Applications:
- Geophysical Surveys: Mapping variations in the Earth's magnetic field to locate minerals.
- Navigation: Compasses in smartphones and GPS systems.
- Military: Submarine detection and mine detection.
"""

add_content("Magnetometer", magnetometer_intro, magnetometer_principles, magnetometer_applications)

# GPS content
gps_intro = """
The Global Positioning System (GPS) is a satellite-based navigation system that provides location and time information anywhere on Earth.
It is maintained by the United States government and freely accessible to anyone with a GPS receiver.
"""

gps_principles = """
GPS works by receiving signals from a network of satellites:
- Triangulation: By calculating the time it takes for signals to travel from multiple satellites, a GPS receiver can determine its location.
- Satellite Constellation: At least 24 satellites in medium Earth orbit, ensuring at least 4 satellites are visible from any point on Earth.
- Error Correction: Techniques like Differential GPS (DGPS) and Assisted GPS (A-GPS) improve accuracy.
"""

gps_applications = """
Applications:
- Navigation: Used in cars, aircraft, ships, and smartphones.
- Timing: Synchronizing time in networks and financial systems.
- Geolocation: Location-based services, asset tracking, and geocaching.
"""

add_content("GPS", gps_intro, gps_principles, gps_applications)

# RTK GPS content
rtk_intro = """
Real-Time Kinematic (RTK) GPS is a satellite navigation technique used to enhance the precision of position data derived from satellite-based positioning systems.
RTK provides real-time corrections, delivering centimeter-level accuracy.
"""

rtk_principles = """
RTK uses a fixed base station and a rover:
- Base Station: Positioned at a known location, it continuously measures the satellite signals.
- Rover: Receives signals from both the satellites and the base station.
- Corrections: The base station calculates corrections and transmits them to the rover, which applies these corrections to its own position calculations.
"""

rtk_applications = """
Applications:
- Surveying: High-precision mapping and land surveying.
- Agriculture: Precision farming techniques.
- Construction: Accurate positioning of machinery and materials.
"""

add_content("RTK GPS", rtk_intro, rtk_principles, rtk_applications)

# BLDC Motor content
bldc_intro = """
A Brushless DC (BLDC) motor is an electric motor driven by direct current (DC) electricity and having an electronically controlled commutation system, instead of a mechanical commutation system.
BLDC motors are known for their efficiency and reliability.
"""

bldc_principles = """
BLDC motors use electronic controllers:
- Stator and Rotor: The stator contains the windings, and the rotor is fitted with permanent magnets.
- Commutation: Electronic commutation replaces the mechanical brushes and commutators, using semiconductor switches to energize the windings.
- Sensors: Hall effect sensors or back EMF (electromotive force) detection is used to determine rotor position and control the commutation sequence.
"""

bldc_applications = """
Applications:
- Automotive: Electric vehicles, power steering, and HVAC systems.
- Industrial: Automation systems, CNC machines, and robotics.
- Consumer Electronics: Hard drives, cooling fans, and drones.
"""

add_content("BLDC Motor", bldc_intro, bldc_principles, bldc_applications)

# Save the PDF
pdf_file_path = "/mnt/data/Sensor_Technology_Report.pdf"
pdf.output(pdf_file_path)
pdf_file_path
``` &#8203;:citation[oaicite:0]{index=0}&#8203;
