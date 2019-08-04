Thank you for contributing to Project OpaNoid! The following is a set
of guidelines for contributing to Project
OpaNoid. Above all, these are guidelines, not rules. Use your
best judgement, and feel free to propose changes to this document through
a pull request. 

## Getting started

### Design Considerations of Project OpaNoid

These design considerations serve as
guidelines to increase the ease with which you can integrate your hardware contributions
with the project, and are by no means* *supposed to be definitive.

### Design Ideology (as stated in Readme)

-   By focusing on the humanoid form, greater attention can be paid to the key
    objectives of Project Opanoid; namely its variable mass distribution,
    variable dimensions, scalability (servos can be stacked to support higher
    torque requirements) and expandability (add-ons such as sensors can be added
    without modification to the core structure through the use of rail
    mountings)

### Designing for FDM printing

-   Certain parts have had a portion of their bases raised to facilitate removal
    from the build plate

-   Parts are shaped such that support material required to print them is
    minimised

-   All rail guides along with most joints/pins are designed to be printed
    vertically to increase accuracy

-   Tolerances are mostly rather generous to take into account 3D printers that
    are unable to achieve high levels of refinement

-   Parts are are composed of flat surfaces to reduce printing time, with the
    exception of areas where contouring would be functional, such as the feet

-   Parts are engineered with the material limits of 3D printed PLA in mind

-   Certain parts with very small bases (e.g. the rails) have flaps on the
    bottom in order to increase base area for better adhesion to the build
    plate-these flaps can be broken off the main part relatively cleanly without
    the use of tools

-   As far as possible, parts are designed to be symmetrical along the vertical
    and horizontal axes in order to increase reusability and avoid waste
    materials and time (you don't want to spend hours printing a part for the
    left knee only to realise it was the right knee that needed the part!)

### Functional Considerations

-   Parts are designed to be "open" to increase accessibility during assembly
    and minimise mass

-   With the exception of certain areas, most screws are designed to be used
    without nuts-the threads created by the screw in the plastic through
    self-taping is sufficient to hold the screw in place

-   Brackets are designed such that the rails can extend through most of their
    length, enabling the mounting brackets to be positioned along the rails with
    fewer restrictions

-   The thickness of the part walls are minimised (taking into consideration the
    frailties of the construction material) in order to minimise mass, such that
    the robot's frame constitutes as small a component of the overall mass as
    possible

    -   This would allow the mass distribution of the robot to be controlled
        with greater authority through the positioning of servos and electronics
        within the frame

### Technical documentation 

-   Please refer to the *technical documentation* for schematics of the
    OpaRail system and rail spacings within the limbs and torso 

### How do I contribute?

#### Hardware

-   Due to the modularity of Project OpaNoid, it is possible to add new ideas
    without overriding existing ones

-   Since we would like to encourage variety, we
    prefer not to delete existing ideas (they may have elements which
    we can still learn from!)

-   Hence, contributors are encouraged to add to, rather than replace existing
    material

-   In your issue 

-   If you intend to override an existing file with a new one, please
    refrain from deleting it in your fork. Instead, append your contribution
    to the existing project as per the default procedure but mention the
    file you wish to replace in the pull request, and we
    will review your request 

    -   We may decide to add your contribution but preserve the file you wish to replace 

#### Software

-   The software for Project OpaNoid is at a
    lower developmental state than the hardware. As such, contributions to flesh
    out the software, in particular the controller, are most welcome.

-   Create a pull request with a general description of your modifications, and
    we will review your contribution 
