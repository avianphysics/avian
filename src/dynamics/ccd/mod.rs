//! Components and functionality for Continuous Collision Detection (CCD).
//!
//! # What Is CCD?
//!
//! Physics simulation is typically done in a discrete manner. At the beginning
//! of each timestep, the simulation checks for collisions, and if none are found
//! for a given rigid body, it is free to move according to its velocity and the size
//! of the timestep.
//!
//! This generally works well for large or slowly moving objects, but fast and small
//! objects can pass through thin geometry such as walls and triangle meshes.
//! This phenomenon is often called **tunneling**.
//!
//! <svg width="300" height="350" viewBox="0 0 300 350" fill="none" xmlns="http://www.w3.org/2000/svg">
//!     <rect x="141" y="1" width="18" height="298" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="275" cy="150" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="25" cy="150" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <path d="M275.707 150.707C276.098 150.317 276.098 149.683 275.707 149.293L269.343 142.929C268.953 142.538 268.319 142.538 267.929 142.929C267.538 143.319 267.538 143.953 267.929 144.343L273.586 150L267.929 155.657C267.538 156.047 267.538 156.681 267.929 157.071C268.319 157.462 268.953 157.462 269.343 157.071L275.707 150.707ZM25 151L275 151L275 149L25 149L25 151Z" fill="#E19664"/>
//!     <text x="150" y="325" style="fill: #b4b4b4; font: 18px monospace; text-anchor: middle;">Discrete</text>
//! </svg>
//!
//! **Continuous Collision Detection (CCD)** aims to prevent tunneling and deep overlap.
//! Currently, two approaches are supported: [swept CCD](#swept-ccd) and
//! [speculative collision](#speculative-collision). Swept CCD is performed for fast-moving
//! dynamic bodies *by default*, and can be configured using the [`SweptCcd`] component,
//! while speculative collision is fully opt-in via the [`SpeculativeCcd`] component.
//!
//! ## Swept CCD
//!
//! *Note: Swept CCD currently only supports the built-in `Collider`.*
//!
//! **Swept CCD** is a form of Continuous Collision Detection that sweeps fast-moving dynamic bodies
//! from their previous positions to their current positions, and if a collision is found along the path,
//! moves the body back to the time of impact. This way, discrete collision algorithms will be able to detect
//! and handle the collision during the next timestep.
//!
//! There are two variants of swept CCD: [`SweepMode::Linear`] and [`SweepMode::NonLinear`].
//! The difference is that [`Linear`](SweepMode::Linear) only considers translational motion,
//! so fast-spinning bodies can still pass through objects, while [`NonLinear`](SweepMode::NonLinear)
//! also considers rotational motion, but is more expensive.
//!
//! <svg width="300" height="350" viewBox="0 0 300 350" fill="none" xmlns="http://www.w3.org/2000/svg">
//!     <rect x="141" y="1" width="18" height="298" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="115" cy="150" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="25" cy="150" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <path d="M115.707 150.707C116.098 150.317 116.098 149.683 115.707 149.293L109.343 142.929C108.953 142.538 108.319 142.538 107.929 142.929C107.538 143.319 107.538 143.953 107.929 144.343L113.586 150L107.929 155.657C107.538 156.047 107.538 156.681 107.929 157.071C108.319 157.462 108.953 157.462 109.343 157.071L115.707 150.707ZM25 151H115V149H25V151Z" fill="#E19664"/>
//!     <text x="150" y="325" style="fill: #b4b4b4; font: 18px monospace; text-anchor: middle;">Linear / NonLinear</text>
//! </svg>
//!
//! <svg width="600" height="350" viewBox="0 0 600 350" fill="none" xmlns="http://www.w3.org/2000/svg">
//!     <rect x="438.95" y="25.3488" width="18" height="298" transform="rotate(27.5 438.95 25.3488)" stroke="#64C850" stroke-width="2" stroke-dasharray="4 4"/>
//!     <rect x="301" y="1" width="18" height="298" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="425" cy="150" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <text x="150" y="325" style="fill: #b4b4b4; font: 18px monospace; text-anchor: middle;">Linear</text>
//!     <circle cx="310" cy="290" r="5" fill="#D9D9D9" stroke="black" stroke-width="2"/>
//!     <path d="M322.96 148.816L323.331 149.745L323.331 149.745L322.96 148.816ZM361.786 156.786L361.078 157.493L361.078 157.493L361.786 156.786ZM365 161C365.552 161 366 160.552 366 160L366 151C366 150.448 365.552 150 365 150C364.448 150 364 150.448 364 151L364 159L356 159C355.448 159 355 159.448 355 160C355 160.552 355.448 161 356 161L365 161ZM320.371 150.928L323.331 149.745L322.588 147.888L319.629 149.072L320.371 150.928ZM361.078 157.493L364.293 160.707L365.707 159.293L362.493 156.078L361.078 157.493ZM323.331 149.745C336.331 144.545 351.178 147.592 361.078 157.493L362.493 156.078C352.027 145.612 336.331 142.391 322.588 147.888L323.331 149.745Z" fill="#E19664"/>
//!     <rect x="259.442" y="133.366" width="18" height="298" transform="rotate(60 259.442 133.366)" stroke="#64C850" stroke-width="2" stroke-dasharray="4 4"/>
//!     <rect x="1" y="1" width="18" height="298" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="125" cy="150" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <text x="450" y="325" style="fill: #b4b4b4; font: 18px monospace; text-anchor: middle;">NonLinear</text>
//!     <circle cx="10" cy="290" r="5" fill="#D9D9D9" stroke="black" stroke-width="2"/>
//!     <path d="M54.0561 245.357L53.1144 245.694L53.1144 245.694L54.0561 245.357ZM54.5719 248.904C55.071 249.14 55.6673 248.927 55.9037 248.428L59.7565 240.294C59.9929 239.795 59.7799 239.199 59.2808 238.963C58.7817 238.726 58.1854 238.939 57.949 239.438L54.5243 246.668L47.2944 243.243C46.7953 243.007 46.199 243.22 45.9626 243.719C45.7261 244.218 45.9391 244.815 46.4382 245.051L54.5719 248.904ZM53.1144 245.694L54.0582 248.336L55.9417 247.664L54.9978 245.021L53.1144 245.694ZM20.3714 230.928C33.4979 225.678 48.3594 232.379 53.1144 245.694L54.9978 245.021C49.8615 230.639 33.808 223.4 19.6286 229.071L20.3714 230.928Z" fill="#E19664"/>
//! </svg>
//!
//! Sweeps are performed **automatically** for every *fast* dynamic body, against both static
//! and kinematic bodies. A body is considered "fast" if its motion during a timestep is large
//! relative to its minimum thickness. This way, we only pay the cost of CCD for bodies that
//! have a risk of tunneling or deep overlap.
//!
//! The behavior of swept CCD can be customized using the [`SweptCcd`] component.
//! This includes configuring which types of bodies sweeps are performed against,
//! tuning the threshold for what is considered a "fast" body, and changing
//! the sweep mode:
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! fn setup(mut commands: Commands) {
//!     commands.spawn((
//!         RigidBody::Dynamic,
//!         Collider::capsule(0.5, 2.0),
//!         // These are the default settings
//!         SweptCcd {
//!             filter: CcdFilter::DEFAULT,
//!             mode: SweepMode::NonLinear,
//!             threshold: 0.5,
//!         }
//!     ));
//! }
//! ```
//!
//! See the documentation of the [`SweptCcd`] component for more details
//! and configuration options.
//!
//! This approach to CCD is heavily inspired by [Box2D](https://box2d.org) by Erin Catto.
//!
//! ### Caveats of Swept CCD
//!
//! Swept CCD can lead to *time loss* or *time stealing*, where bodies appear to momentarily
//! slow down or freeze when colliding at high speeds. This happens because they are essentially
//! moved backwards in time to avoid missing the collision. Time loss could be better accounted for
//! with a substepped time-of-impact solver, but it would be much more expensive and complex,
//! so it is not supported yet.
//!
//! Additionally, if two fast-moving bodies are approaching each other, but are not already
//! in the path of each other's sweeps (ex: projectiles coming from different directions),
//! they can miss each other. This can be solved by also enabling [speculative collision](#speculative-collision)
//! in order to expand their AABBs based on velocity and predict contacts before they happen.
//!
//! ## Speculative Collision
//!
//! **Speculative collision** is a form of Continuous Collision Detection
//! where contacts are predicted before they happen. It is only performed
//! for bodies that have the [`SpeculativeCcd`] component.
//!
//! To determine whether two bodies may come into contact within the next timestep,
//! speculative collision expands their [`ColliderAabb`]s based on their velocities.
//! Additionally, a **speculative margin** is used to determine the maximum distance
//! at which two shapes can generate speculative contacts.
//!
//! <svg width="335" height="400" viewBox="0 0 335 400" fill="none" xmlns="http://www.w3.org/2000/svg">
//!     <rect x="36" y="266" width="298" height="18" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="210" cy="375" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="210" cy="240" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="160" cy="160" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <path d="M209.471 375.849C209.94 376.141 210.557 375.997 210.849 375.529L215.606 367.888C215.898 367.42 215.754 366.803 215.286 366.511C214.817 366.219 214.2 366.363 213.908 366.831L209.68 373.623L202.888 369.394C202.42 369.102 201.803 369.246 201.511 369.714C201.219 370.183 201.363 370.8 201.831 371.092L209.471 375.849ZM159.026 160.227L159.472 162.146L161.42 161.693L160.974 159.773L159.026 160.227ZM160.365 165.985L161.258 169.825L163.206 169.372L162.313 165.532L160.365 165.985ZM162.151 173.664L163.044 177.503L164.992 177.05L164.099 173.211L162.151 173.664ZM163.937 181.343L164.83 185.182L166.778 184.729L165.885 180.89L163.937 181.343ZM165.722 189.021L166.615 192.86L168.563 192.407L167.67 188.568L165.722 189.021ZM167.508 196.7L168.401 200.539L170.349 200.086L169.456 196.247L167.508 196.7ZM169.294 204.378L170.187 208.218L172.135 207.765L171.242 203.925L169.294 204.378ZM171.08 212.057L171.972 215.896L173.92 215.443L173.028 211.604L171.08 212.057ZM172.865 219.735L173.758 223.575L175.706 223.122L174.813 219.282L172.865 219.735ZM174.651 227.414L175.544 231.253L177.492 230.8L176.599 226.961L174.651 227.414ZM176.437 235.093L177.33 238.932L179.278 238.479L178.385 234.64L176.437 235.093ZM178.222 242.771L179.115 246.61L181.063 246.157L180.17 242.318L178.222 242.771ZM180.008 250.45L180.901 254.289L182.849 253.836L181.956 249.997L180.008 250.45ZM181.794 258.128L182.687 261.968L184.635 261.515L183.742 257.675L181.794 258.128ZM183.58 265.807L184.472 269.646L186.42 269.193L185.528 265.354L183.58 265.807ZM185.365 273.485L186.258 277.325L188.206 276.872L187.313 273.032L185.365 273.485ZM187.151 281.164L188.044 285.003L189.992 284.55L189.099 280.711L187.151 281.164ZM188.937 288.843L189.83 292.682L191.778 292.229L190.885 288.39L188.937 288.843ZM190.722 296.521L191.615 300.36L193.563 299.907L192.67 296.068L190.722 296.521ZM192.508 304.2L193.401 308.039L195.349 307.586L194.456 303.747L192.508 304.2ZM194.294 311.878L195.187 315.718L197.135 315.265L196.242 311.425L194.294 311.878ZM196.08 319.557L196.972 323.396L198.92 322.943L198.028 319.104L196.08 319.557ZM197.865 327.235L198.758 331.075L200.706 330.622L199.813 326.782L197.865 327.235ZM199.651 334.914L200.544 338.753L202.492 338.3L201.599 334.461L199.651 334.914ZM201.437 342.593L202.33 346.432L204.278 345.979L203.385 342.14L201.437 342.593ZM203.222 350.271L204.115 354.111L206.063 353.657L205.17 349.818L203.222 350.271ZM205.008 357.95L205.901 361.789L207.849 361.336L206.956 357.497L205.008 357.95ZM206.794 365.628L207.687 369.468L209.635 369.015L208.742 365.175L206.794 365.628ZM208.58 373.307L209.026 375.226L210.974 374.773L210.528 372.854L208.58 373.307ZM209.471 375.849C209.94 376.141 210.557 375.997 210.849 375.529L215.606 367.888C215.898 367.42 215.754 366.803 215.286 366.511C214.817 366.219 214.2 366.363 213.908 366.831L209.68 373.623L202.888 369.394C202.42 369.102 201.803 369.246 201.511 369.714C201.219 370.183 201.363 370.8 201.831 371.092L209.471 375.849ZM159.026 160.227L159.472 162.146L161.42 161.693L160.974 159.773L159.026 160.227ZM160.365 165.985L161.258 169.825L163.206 169.372L162.313 165.532L160.365 165.985ZM162.151 173.664L163.044 177.503L164.992 177.05L164.099 173.211L162.151 173.664ZM163.937 181.343L164.83 185.182L166.778 184.729L165.885 180.89L163.937 181.343ZM165.722 189.021L166.615 192.86L168.563 192.407L167.67 188.568L165.722 189.021ZM167.508 196.7L168.401 200.539L170.349 200.086L169.456 196.247L167.508 196.7ZM169.294 204.378L170.187 208.218L172.135 207.765L171.242 203.925L169.294 204.378ZM171.08 212.057L171.972 215.896L173.92 215.443L173.028 211.604L171.08 212.057ZM172.865 219.735L173.758 223.575L175.706 223.122L174.813 219.282L172.865 219.735ZM174.651 227.414L175.544 231.253L177.492 230.8L176.599 226.961L174.651 227.414ZM176.437 235.093L177.33 238.932L179.278 238.479L178.385 234.64L176.437 235.093ZM178.222 242.771L179.115 246.61L181.063 246.157L180.17 242.318L178.222 242.771ZM180.008 250.45L180.901 254.289L182.849 253.836L181.956 249.997L180.008 250.45ZM181.794 258.128L182.687 261.968L184.635 261.515L183.742 257.675L181.794 258.128ZM183.58 265.807L184.472 269.646L186.42 269.193L185.528 265.354L183.58 265.807ZM185.365 273.485L186.258 277.325L188.206 276.872L187.313 273.032L185.365 273.485ZM187.151 281.164L188.044 285.003L189.992 284.55L189.099 280.711L187.151 281.164ZM188.937 288.843L189.83 292.682L191.778 292.229L190.885 288.39L188.937 288.843ZM190.722 296.521L191.615 300.36L193.563 299.907L192.67 296.068L190.722 296.521ZM192.508 304.2L193.401 308.039L195.349 307.586L194.456 303.747L192.508 304.2ZM194.294 311.878L195.187 315.718L197.135 315.265L196.242 311.425L194.294 311.878ZM196.08 319.557L196.972 323.396L198.92 322.943L198.028 319.104L196.08 319.557ZM197.865 327.235L198.758 331.075L200.706 330.622L199.813 326.782L197.865 327.235ZM199.651 334.914L200.544 338.753L202.492 338.3L201.599 334.461L199.651 334.914ZM201.437 342.593L202.33 346.432L204.278 345.979L203.385 342.14L201.437 342.593ZM203.222 350.271L204.115 354.111L206.063 353.657L205.17 349.818L203.222 350.271ZM205.008 357.95L205.901 361.789L207.849 361.336L206.956 357.497L205.008 357.95ZM206.794 365.628L207.687 369.468L209.635 369.015L208.742 365.175L206.794 365.628ZM208.58 373.307L209.026 375.226L210.974 374.773L210.528 372.854L208.58 373.307Z" fill="#AF644B"/>
//!     <path d="M209.775 240.974C210.313 241.099 210.85 240.763 210.974 240.225L212.998 231.455C213.122 230.917 212.787 230.38 212.249 230.256C211.71 230.132 211.174 230.467 211.049 231.006L209.25 238.801L201.455 237.002C200.917 236.878 200.38 237.213 200.256 237.751C200.132 238.29 200.467 238.826 201.006 238.951L209.775 240.974ZM159.152 160.53L209.152 240.53L210.848 239.47L160.848 159.47L159.152 160.53Z" fill="#E19664"/>
//!     <circle cx="160" cy="265" r="4" fill="#a874d8"/>
//!     <path d="M160 160V265" stroke="#a874d8" stroke-width="2" stroke-dasharray="5 5"/>
//!     <rect x="135" y="135" width="99" height="264" stroke="#db9010" stroke-width="1"/>
//!     <circle cx="160" cy="160" r="159" stroke="#eb4a5a" stroke-width="1"/>
//!     <text x="184" y="125" style="fill: #db9010; font: 16px monospace; text-anchor: middle;">Collision AABB</text>
//!     <text x="160" y="40" style="fill: #eb4a5a; font: 16px monospace; text-anchor: middle;">Speculative Margin</text>
//!     <text x="210" y="340" style="fill: #b4b4b4; font: 16px monospace; text-anchor: start;">Unconstrained</text>
//!     <text x="210" y="205" style="fill: #b4b4b4; font: 16px monospace; text-anchor: start;">Constrained</text>
//!     <text y="240" style="fill: #a874d8; font: 16px monospace; font-weight: bold; text-anchor: end;">
//!         <tspan x="155">Speculative</tspan><tspan x="155" dy="18">Contact</tspan>
//!     </text>
//! </svg>
//!
//! By default, Avian keeps AABBs tight, and only predicts contacts up to a small,
//! fixed [`contact_tolerance`], which aims to improve stability and avoid jitter
//! for resting bodies. By adding the [`SpeculativeCcd`] component, you can increase
//! the maximum speculative distance and opt in to velocity-based AABB expansion,
//! enabling speculative contact behavior.
//!
//! Speculative collisions can be very efficient for highly dynamic scenes,
//! but a large speculative margin can also lead to excessive collision tests
//! and false positives, which are described below.
//!
//! [`contact_tolerance`]: NarrowPhaseConfig::contact_tolerance
//!
//! ### Caveats of Speculative Collision
//!
//! Speculative contacts are approximations. They typically have good enough accuracy,
//! but when bodies are moving past each other at high speeds, the prediction can sometimes
//! fail and lead to **ghost collisions**. This happens because contact surfaces are treated
//! like infinite planes from the point of view of the contact solver. Ghost collisions
//! typically manifest as objects bumping into seemingly invisible walls or seams.
//!
//! <svg width="475" height="400" viewBox="0 0 475 400" fill="none" xmlns="http://www.w3.org/2000/svg">
//!     <rect x="36" y="266" width="148" height="18" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="345" cy="375" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="345" cy="236" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="160" cy="160" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <path d="M344.925 375.997C345.476 376.038 345.956 375.626 345.997 375.075L346.67 366.1C346.712 365.549 346.299 365.069 345.748 365.028C345.197 364.987 344.717 365.4 344.676 365.95L344.078 373.928L336.1 373.33C335.549 373.288 335.069 373.701 335.028 374.252C334.987 374.803 335.4 375.283 335.95 375.324L344.925 375.997ZM159.242 160.652L160.563 162.188L162.079 160.883L160.758 159.348L159.242 160.652ZM163.206 165.259L165.849 168.331L167.365 167.026L164.722 163.955L163.206 165.259ZM168.492 171.402L171.135 174.474L172.651 173.169L170.008 170.098L168.492 171.402ZM173.778 177.545L176.421 180.617L177.937 179.312L175.294 176.241L173.778 177.545ZM179.063 183.688L181.706 186.759L183.222 185.455L180.579 182.383L179.063 183.688ZM184.349 189.831L186.992 192.902L188.508 191.598L185.865 188.526L184.349 189.831ZM189.635 195.974L192.278 199.045L193.794 197.741L191.151 194.669L189.635 195.974ZM194.921 202.117L197.563 205.188L199.079 203.883L196.437 200.812L194.921 202.117ZM200.206 208.259L202.849 211.331L204.365 210.026L201.722 206.955L200.206 208.259ZM205.492 214.402L208.135 217.474L209.651 216.169L207.008 213.098L205.492 214.402ZM210.778 220.545L213.421 223.617L214.937 222.312L212.294 219.241L210.778 220.545ZM216.063 226.688L218.706 229.759L220.222 228.455L217.579 225.383L216.063 226.688ZM221.349 232.831L223.992 235.902L225.508 234.598L222.865 231.526L221.349 232.831ZM226.635 238.974L229.278 242.045L230.794 240.741L228.151 237.669L226.635 238.974ZM231.921 245.117L234.563 248.188L236.079 246.883L233.437 243.812L231.921 245.117ZM237.206 251.259L239.849 254.331L241.365 253.026L238.722 249.955L237.206 251.259ZM242.492 257.402L245.135 260.474L246.651 259.169L244.008 256.098L242.492 257.402ZM247.778 263.545L250.421 266.617L251.937 265.312L249.294 262.241L247.778 263.545ZM253.063 269.688L255.706 272.759L257.222 271.455L254.579 268.383L253.063 269.688ZM258.349 275.831L260.992 278.902L262.508 277.598L259.865 274.526L258.349 275.831ZM263.635 281.974L266.278 285.045L267.794 283.741L265.151 280.669L263.635 281.974ZM268.921 288.116L271.563 291.188L273.079 289.883L270.437 286.812L268.921 288.116ZM274.206 294.259L276.849 297.331L278.365 296.026L275.722 292.955L274.206 294.259ZM279.492 300.402L282.135 303.474L283.651 302.169L281.008 299.098L279.492 300.402ZM284.778 306.545L287.421 309.616L288.937 308.312L286.294 305.241L284.778 306.545ZM290.063 312.688L292.706 315.759L294.222 314.455L291.579 311.383L290.063 312.688ZM295.349 318.831L297.992 321.902L299.508 320.598L296.865 317.526L295.349 318.831ZM300.635 324.974L303.278 328.045L304.794 326.741L302.151 323.669L300.635 324.974ZM305.921 331.116L308.563 334.188L310.079 332.883L307.437 329.812L305.921 331.116ZM311.206 337.259L313.849 340.331L315.365 339.026L312.722 335.955L311.206 337.259ZM316.492 343.402L319.135 346.474L320.651 345.169L318.008 342.098L316.492 343.402ZM321.778 349.545L324.421 352.616L325.937 351.312L323.294 348.241L321.778 349.545ZM327.063 355.688L329.706 358.759L331.222 357.455L328.579 354.383L327.063 355.688ZM332.349 361.831L334.992 364.902L336.508 363.598L333.865 360.526L332.349 361.831ZM337.635 367.974L340.278 371.045L341.794 369.741L339.151 366.669L337.635 367.974ZM342.921 374.117L344.242 375.652L345.758 374.348L344.437 372.812L342.921 374.117ZM344.925 375.997C345.476 376.038 345.956 375.626 345.997 375.075L346.67 366.1C346.712 365.549 346.299 365.069 345.748 365.028C345.197 364.987 344.717 365.4 344.676 365.95L344.078 373.928L336.1 373.33C335.549 373.288 335.069 373.701 335.028 374.252C334.987 374.803 335.4 375.283 335.95 375.324L344.925 375.997ZM159.242 160.652L160.563 162.188L162.079 160.883L160.758 159.348L159.242 160.652ZM163.206 165.259L165.849 168.331L167.365 167.026L164.722 163.955L163.206 165.259ZM168.492 171.402L171.135 174.474L172.651 173.169L170.008 170.098L168.492 171.402ZM173.778 177.545L176.421 180.617L177.937 179.312L175.294 176.241L173.778 177.545ZM179.063 183.688L181.706 186.759L183.222 185.455L180.579 182.383L179.063 183.688ZM184.349 189.831L186.992 192.902L188.508 191.598L185.865 188.526L184.349 189.831ZM189.635 195.974L192.278 199.045L193.794 197.741L191.151 194.669L189.635 195.974ZM194.921 202.117L197.563 205.188L199.079 203.883L196.437 200.812L194.921 202.117ZM200.206 208.259L202.849 211.331L204.365 210.026L201.722 206.955L200.206 208.259ZM205.492 214.402L208.135 217.474L209.651 216.169L207.008 213.098L205.492 214.402ZM210.778 220.545L213.421 223.617L214.937 222.312L212.294 219.241L210.778 220.545ZM216.063 226.688L218.706 229.759L220.222 228.455L217.579 225.383L216.063 226.688ZM221.349 232.831L223.992 235.902L225.508 234.598L222.865 231.526L221.349 232.831ZM226.635 238.974L229.278 242.045L230.794 240.741L228.151 237.669L226.635 238.974ZM231.921 245.117L234.563 248.188L236.079 246.883L233.437 243.812L231.921 245.117ZM237.206 251.259L239.849 254.331L241.365 253.026L238.722 249.955L237.206 251.259ZM242.492 257.402L245.135 260.474L246.651 259.169L244.008 256.098L242.492 257.402ZM247.778 263.545L250.421 266.617L251.937 265.312L249.294 262.241L247.778 263.545ZM253.063 269.688L255.706 272.759L257.222 271.455L254.579 268.383L253.063 269.688ZM258.349 275.831L260.992 278.902L262.508 277.598L259.865 274.526L258.349 275.831ZM263.635 281.974L266.278 285.045L267.794 283.741L265.151 280.669L263.635 281.974ZM268.921 288.116L271.563 291.188L273.079 289.883L270.437 286.812L268.921 288.116ZM274.206 294.259L276.849 297.331L278.365 296.026L275.722 292.955L274.206 294.259ZM279.492 300.402L282.135 303.474L283.651 302.169L281.008 299.098L279.492 300.402ZM284.778 306.545L287.421 309.616L288.937 308.312L286.294 305.241L284.778 306.545ZM290.063 312.688L292.706 315.759L294.222 314.455L291.579 311.383L290.063 312.688ZM295.349 318.831L297.992 321.902L299.508 320.598L296.865 317.526L295.349 318.831ZM300.635 324.974L303.278 328.045L304.794 326.741L302.151 323.669L300.635 324.974ZM305.921 331.116L308.563 334.188L310.079 332.883L307.437 329.812L305.921 331.116ZM311.206 337.259L313.849 340.331L315.365 339.026L312.722 335.955L311.206 337.259ZM316.492 343.402L319.135 346.474L320.651 345.169L318.008 342.098L316.492 343.402ZM321.778 349.545L324.421 352.616L325.937 351.312L323.294 348.241L321.778 349.545ZM327.063 355.688L329.706 358.759L331.222 357.455L328.579 354.383L327.063 355.688ZM332.349 361.831L334.992 364.902L336.508 363.598L333.865 360.526L332.349 361.831ZM337.635 367.974L340.278 371.045L341.794 369.741L339.151 366.669L337.635 367.974ZM342.921 374.117L344.242 375.652L345.758 374.348L344.437 372.812L342.921 374.117Z" fill="#AF644B"/>
//!     <path d="M345.385 236.923C345.895 236.71 346.136 236.124 345.923 235.615L342.454 227.31C342.242 226.8 341.656 226.56 341.146 226.772C340.637 226.985 340.396 227.571 340.609 228.08L343.692 235.463L336.31 238.546C335.8 238.758 335.56 239.344 335.772 239.854C335.985 240.363 336.571 240.604 337.08 240.391L345.385 236.923ZM159.62 160.925L344.62 236.925L345.38 235.075L160.38 159.075L159.62 160.925Z" fill="#E19664"/>
//!     <circle cx="160" cy="265" r="4" fill="#a874d8"/>
//!     <path d="M160 160V265" stroke="#a874d8" stroke-width="2" stroke-dasharray="5 5"/>
//!     <rect x="135" y="135" width="235" height="264" stroke="#db9010" stroke-width="1"/>
//!     <circle cx="160" cy="160" r="159" stroke="#eb4a5a" stroke-width="1"/>
//!     <line x1="160" y1="264" x2="400" y2="264" stroke="#a874d8" stroke-width="2" stroke-dasharray="6 6"/>
//!     <text x="184" y="125" style="fill: #db9010; font: 16px monospace; text-anchor: middle;">Collision AABB</text>
//!     <text x="160" y="40" style="fill: #eb4a5a; font: 16px monospace; text-anchor: middle;">Speculative Margin</text>
//!     <text x="345" y="340" style="fill: #b4b4b4; font: 16px monospace; text-anchor: start;">Unconstrained</text>
//!     <text x="345" y="205" style="fill: #b4b4b4; font: 16px monospace; text-anchor: start;">Constrained</text>
//!     <text x="190" y="280" style="fill: #a874d8; font: 16px monospace; font-weight: bold; text-anchor: start;">Ghost Collision Plane</text>
//! </svg>
//!
//! Another caveat of speculative collision is that it can still occasionally miss contacts,
//! especially for fast-spinning objects, as speculative contacts do not properly account for
//! rotational motion.
//!
//! Speculative collisions can also absorb some energy in contacts, causing even perfectly elastic
//! objects to lose kinetic energy over several bounces.
//!
//! These caveats together with the performance impact of velocity-expanded AABBs
//! are why Avian does not predict contacts beyond the small [`contact_tolerance`]
//! by default. Still, speculative contacts can be a good option for some scenes,
//! which is what the opt-in [`SpeculativeCcd`] component is for.
//!
//! ## Other Ways to Avoid Tunneling
//!
//! CCD is one way to prevent objects from tunneling through each other,
//! but it should only be used when necessary. There are several other approaches
//! worth considering to help avoid the issue.
//!
//! The most obvious way is to simply avoid small or thin geometry such as triangle meshes,
//! and to make colliders for objects like walls slightly thicker than necessary.
//! This is of course not possible everywhere, but it is good to keep in mind
//! when authoring levels.
//!
//! Triangle mesh colliders are especially prone to tunneling for dynamic rigid bodies.
//! For shapes that are intended to be solid from the inside, it is recommended
//! to use convex decomposition instead.
//!
//! If you must use triangle mesh colliders and are having stability issues, consider
//! giving them a small amount of extra thickness using the [`CollisionMargin`] component.
//! This helps prevent objects from passing through the surface while also reducing
//! numerical errors and improving performance.
//!
//! Finally, making the [physics timestep](Physics) smaller can also help.
//! However, this comes at the cost of worse performance for the entire simulation.

#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use super::solver::solver_body::{SolverBody, SolverBodyFlags};
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use crate::prelude::*;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use crate::{
    collider_tree::{ColliderTree, ColliderTreeProxyFlags, ColliderTrees},
    math::make_pose,
};
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use bevy::ecs::query::QueryData;
use bevy::prelude::*;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use core::cell::RefCell;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use dynamics::solver::SolverDiagnostics;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use parry::query::{
    NonlinearRigidMotion, ShapeCastHit, ShapeCastOptions, cast_shapes, cast_shapes_nonlinear,
};
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use thread_local::ThreadLocal;

/// A plugin for [Continuous Collision Detection (CCD)](self).
pub struct CcdPlugin;

impl Plugin for CcdPlugin {
    #[cfg_attr(
        not(any(feature = "parry-f32", feature = "parry-f64")),
        expect(unused_variables)
    )]
    fn build(&self, app: &mut App) {
        // Get the `PhysicsSchedule`, and panic if it doesn't exist.
        #[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
        let physics = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        #[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
        physics.add_systems(solve_continuous.in_set(SolverSystems::ContinuousCollision));
    }
}

/// A component that configures [swept Continuous Collision Detection (CCD)](self#swept-ccd)
/// for a [dynamic](RigidBody::Dynamic) [`RigidBody`].
///
/// By default, swept CCD is performed automatically for fast-moving dynamic bodies,
/// against static and kinematic bodies but not other dynamic bodies.
///
/// Adding this component allows you to:
///
/// - control which types of bodies sweeps are performed against (via [`filter`](Self::filter))
/// - choose the [sweep mode](SweepMode) (via [`mode`](Self::mode))
/// - adjust the threshold for when CCD is triggered (via [`threshold`](Self::threshold))
///
/// This component has no effect on static or kinematic bodies, as they do not support CCD.
///
/// Read the [module-level documentation](self) for more information about what CCD is,
/// what it is used for, and what limitations and tradeoffs it can have.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // A fast dynamic body is swept against static and kinematic bodies automatically,
///     // so it needs no extra components.
///     commands.spawn((
///         RigidBody::Dynamic,
#[cfg_attr(feature = "2d", doc = "        LinearVelocity(Vec2::X * 100.0),")]
#[cfg_attr(feature = "3d", doc = "        LinearVelocity(Vec3::X * 100.0),")]
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.1),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.1),")]
///         Transform::from_xyz(-10.0, 3.0, 0.0),
///     ));
///
///     // Spawn another body that only considers linear motion and not rotation,
///     // and is additionally swept against other dynamic bodies.
///     commands.spawn((
///         RigidBody::Dynamic,
///         SweptCcd {
///             filter: CcdFilter::ALL,
///             mode: SweepMode::Linear,
///             ..default()
///         },
#[cfg_attr(feature = "2d", doc = "        LinearVelocity(Vec2::X * 100.0),")]
#[cfg_attr(feature = "3d", doc = "        LinearVelocity(Vec3::X * 100.0),")]
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.1),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.1),")]
///         Transform::from_xyz(-10.0, -3.0, 0.0),
///     ));
/// }
/// ```
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[reflect(Component, Debug, Default, PartialEq)]
pub struct SweptCcd {
    /// The types of bodies this body is swept against during CCD.
    ///
    /// Each [`RigidBody`] type can be toggled independently. For example,
    /// fast bodies are swept against static and kinematic bodies by default,
    /// but not against other dynamic bodies.
    ///
    /// If the filter is [empty](CcdFilter::NONE), CCD is disabled for this body entirely.
    /// This can improve performance but can lead to tunneling. Disabling CCD should rarely
    /// be necessary, as it is only performed for fast-moving bodies, and should have a minimal
    /// performance impact for most applications.
    ///
    /// **Default**: [`CcdFilter::DEFAULT`] (static and kinematic bodies)
    pub filter: CcdFilter,

    /// The [sweep mode](SweepMode) used for this body.
    ///
    /// The [`Linear`](SweepMode::Linear) mode is cheaper but only considers translational motion,
    /// so it can lead to tunneling against thin, fast-spinning objects, while the [`NonLinear`](SweepMode::NonLinear)
    /// mode is more expensive but also considers rotational motion.
    ///
    /// If two bodies with different sweep modes collide, [`SweepMode::NonLinear`] is preferred.
    ///
    /// **Default**: [`SweepMode::NonLinear`]
    pub mode: SweepMode,

    /// The fraction of a body's minimum [`ccd_thickness`](BodySizeMetrics::ccd_thickness)
    /// it may travel in a timestep before being treated as a fast-moving body.
    ///
    /// This should typically be in the range `[0.0, 1.0]` to prevent tunneling.
    /// Smaller values trigger CCD more easily, which can help prevent overlap,
    /// at the cost of more overhead as sweeps are performed even at lower speeds.
    ///
    /// **Default**: `0.5`
    pub threshold: f32,
}

impl Default for SweptCcd {
    fn default() -> Self {
        Self::new()
    }
}

impl SweptCcd {
    /// Creates a [`SweptCcd`] configuration with default settings:
    ///
    /// - [`filter`](Self::filter): [`CcdFilter::DEFAULT`] (sweeps against static and kinematic bodies)
    /// - [`mode`](Self::mode): [`SweepMode::NonLinear`]
    /// - [`threshold`](Self::threshold): `0.5`
    #[inline]
    pub const fn new() -> Self {
        Self {
            filter: CcdFilter::DEFAULT,
            mode: SweepMode::NonLinear,
            threshold: 0.5,
        }
    }

    /// Sets the [`CcdFilter`] determining which body types this body is swept against.
    ///
    /// An [empty](CcdFilter::NONE) filter disables CCD for this body entirely.
    #[inline]
    pub const fn with_filter(mut self, filter: CcdFilter) -> Self {
        self.filter = filter;
        self
    }

    /// Sets the [sweep mode](SweepMode) used for this body.
    #[inline]
    pub const fn with_mode(mut self, mode: SweepMode) -> Self {
        self.mode = mode;
        self
    }

    /// Sets the fraction of this body's minimum CCD thickness it may travel in a timestep
    /// before being treated as a fast-moving body.
    #[inline]
    pub const fn with_threshold(mut self, threshold: f32) -> Self {
        self.threshold = threshold;
        self
    }
}

/// A bitmask determining which [`RigidBody`] types a [`SweptCcd`] body is swept against
/// during [Continuous Collision Detection (CCD)](self).
///
/// If [empty](Self::NONE), CCD is disabled for the body entirely.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Reflect)]
#[reflect(Debug, Default, PartialEq)]
pub struct CcdFilter(u8);

bitflags::bitflags! {
    impl CcdFilter: u8 {
        /// Sweep against other dynamic bodies.
        const DYNAMIC = 1 << 0;

        /// Sweep against kinematic bodies.
        const KINEMATIC = 1 << 1;

        /// Sweep against static bodies.
        const STATIC = 1 << 2;

        /// Sweep against standalone colliders (colliders with no rigid body).
        const STANDALONE = 1 << 3;

        /// The default filter: sweep against static and kinematic bodies,
        /// but not against other dynamic bodies or colliders without a body.
        const DEFAULT = Self::KINEMATIC.bits() | Self::STATIC.bits();

        /// Sweep against all body types and standalone colliders.
        const ALL = Self::DYNAMIC.bits()
            | Self::KINEMATIC.bits()
            | Self::STATIC.bits()
            | Self::STANDALONE.bits();

        /// Sweep against no body types, effectively disabling CCD for this body.
        const NONE = 0;
    }
}

impl Default for CcdFilter {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// The algorithm used for sweeps during [Continuous Collision Detection (CCD)](self#swept-ccd).
///
/// If two entities with different sweep modes collide, [`SweepMode::NonLinear`] is preferred.
///
/// The default is [`SweepMode::NonLinear`], which considers both translational
/// and rotational motion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Reflect)]
pub enum SweepMode {
    /// Sweeps are performed using linear time-of-impact queries from the previous positions
    /// of fast bodies to their current positions, stopping the bodies at the first time of impact.
    ///
    /// This mode only considers translational motion, and can lead to tunneling
    /// against thin, fast-spinning objects. For the more expensive version
    /// that also considers rotational motion, consider using [`SweepMode::NonLinear`].
    Linear,

    /// Sweeps are performed using non-linear time-of-impact queries from the previous positions
    /// of fast bodies to their current positions, stopping the bodies at the first time of impact.
    ///
    /// This mode considers both translational and rotational motion.
    /// For the cheaper version that only considers translational motion,
    /// consider using [`SweepMode::Linear`].
    #[default]
    NonLinear,
}

/// A component that enables [speculative collision](self#speculative-collision)
/// and velocity-expanded [AABBs](ColliderAabb) for a [`RigidBody`],
/// allowing it to predict and react to contacts before they happen.
///
/// Without this component, a body still receives a small, fixed [`contact_tolerance`]
/// and automatic [swept CCD](self#swept-ccd), but no velocity-based speculative margin
/// or AABB expansion.
///
/// Speculative collisions can be cheaper than swept CCD for highly dynamic scenes,
/// but are more approximate, and can cause [ghost collisions](self#caveats-of-speculative-collision).
/// The [`max_distance`](Self::max_distance) property bounds the margin to help mitigate this.
///
/// Read the [module-level documentation](self) for more information about what CCD is,
/// what it is used for, and what limitations and tradeoffs it can have.
///
/// [`contact_tolerance`]: NarrowPhaseConfig::contact_tolerance
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // A fast body that predicts contacts up to 2 units ahead based on its velocity,
///     // expands its AABB accordingly, and is also swept against other dynamic bodies.
///     commands.spawn((
///         RigidBody::Dynamic,
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.1),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.1),")]
///         SpeculativeCcd::new(2.0),
///         SweptCcd::default().with_filter(CcdFilter::ALL),
///     ));
/// }
/// ```
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[reflect(Component, Debug, Default, PartialEq)]
#[doc(alias = "SpeculativeMargin")]
pub struct SpeculativeCcd {
    /// The maximum distance at which [speculative contacts](self#speculative-collision)
    /// are generated for this body, and the maximum distance its [`ColliderAabb`] is
    /// expanded along its velocity each timestep.
    ///
    /// Larger values predict contacts further ahead, at the cost of more broad phase work
    /// and a higher risk of [ghost collisions](self#caveats-of-speculative-collision).
    /// A value of `0.0` effectively disables speculative collision for this body.
    ///
    /// **Default**: [`f32::MAX`] (unbounded)
    pub max_distance: f32,
}

impl Default for SpeculativeCcd {
    fn default() -> Self {
        Self::MAX
    }
}

impl SpeculativeCcd {
    /// A zero speculative margin. Disables speculative collision and velocity-based
    /// AABB expansion for this body.
    pub const ZERO: Self = Self::new(0.0);

    /// An unbounded speculative margin. Speculative contacts are generated as far ahead
    /// as the body's velocity reaches, and the AABB is expanded by the full per-timestep motion.
    pub const MAX: Self = Self::new(f32::MAX);

    /// Creates a [`SpeculativeCcd`] configuration with the given maximum speculative distance.
    #[inline]
    pub const fn new(max_distance: f32) -> Self {
        Self { max_distance }
    }
}

/// Read-only [`SolverBody`] motion data used to reconstruct a body's per-timestep sweep.
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
#[derive(QueryData)]
struct CcdBodyQuery {
    entity: Entity,
    body: &'static SolverBody,
    position: &'static Position,
    rotation: &'static Rotation,
    com: &'static ComputedCenterOfMass,
    colliders: &'static RigidBodyColliders,
    size_metrics: &'static BodySizeMetrics,
    ccd: Option<&'static SweptCcd>,
}

/// A time-of-impact result for a single fast body.
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
struct CcdResult {
    /// The fast body that was swept.
    entity: Entity,
    /// The time of impact fraction in `[0, 1]`,
    fraction: f32,
    /// Details of the earliest impact, if one was found.
    impact: Option<CcdImpact>,
}

/// Details of the earliest time-of-impact for a fast body.
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
struct CcdImpact {
    /// The fast body's colliding collider.
    collider1: Entity,
    /// The obstacle's colliding collider.
    collider2: Entity,
    /// Additional speculative distance to request for the discrete solver next timestep,
    /// so that the contact is detected even if the bodies were separated at the TOI impact.
    ///
    /// This is typically only relevant for dynamic-dynamic and dynamic-kinematic impacts
    /// where the final pose of the other shape ended up separated from the TOI impact point.
    distance: f32,
}

/// Performs [Continuous Collision Detection](self).
///
/// Every dynamic body whose per-frame motion is large relative to its thickness is treated
/// as a "fast body" and swept against static and kinematic (and optionally dynamic) bodies.
///
/// If a time of impact is found, the body's accumulated motion is scaled down so it stops
/// at the first impact, leaving the next frame's collision detection to resolve the contact.
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
fn solve_continuous(
    colliders: Query<(&Collider, &Position, &Rotation)>,
    mut bodies: ParamSet<(Query<CcdBodyQuery>, Query<&mut SolverBody>)>,
    trees: Res<ColliderTrees>,
    mut contact_graph: ResMut<ContactGraph>,
    time: Res<Time>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_secs();
    if delta_secs <= 0.0 {
        return;
    }
    let inv_dt = 1.0 / delta_secs;

    // Avian's approach to CCD is heavily inspired by Box2D by Erin Catto,
    // with some notable differences (as of Box2D 3.1.0):
    //
    // - We perform sweeps against both static *and* kinematic bodies by default.
    // - We support sweeps between two dynamic bodies (CCD between "bullets").
    // - We use a bitmask to configure which body types fast bodies are swept against.
    // - We additionally support opt-in Bepu-style speculative collision with a configurable margin.
    // - We run swept CCD before applying solver body deltas. This avoids having to store
    //   previous transforms.
    // - We create contact edges immediately at the time of impact, and request an additional
    //   speculative distance for the narrow phase if needed to ensure the contact is handled.
    //   This fixed some pathological cases with CCD against moving objects.
    // - Miscellaneous differences in structure, naming, and configuration.

    // Per-thread time-of-impact fractions (in `[0, 1]`) for each fast body. A fraction of `1.0`
    // means the body is fast but no impact was found.
    let thread_local_results: ThreadLocal<RefCell<Vec<CcdResult>>> = ThreadLocal::new();

    // First, detect fast bodies and compute their time of impact,
    // collecting the results into `thread_local_results`.
    //
    // This runs in parallel over solver bodies. The order of TOI sweeps does not matter,
    // as each fast body only reads the trees and produces its own deterministic result.
    // This combines parts of `b2FinalizeBodiesTask` and `b2SolveContinuous` from Box2D.
    let ccd_query = bodies.p0();
    ccd_query.par_iter().for_each(|fast| {
        // Only dynamic bodies support CCD.
        if !fast.body.flags.is_dynamic() {
            return;
        }

        let ccd = fast.ccd.copied().unwrap_or_default();

        if ccd.filter.is_empty() {
            return;
        }

        let body = fast.body;
        let ccd_thickness = fast.size_metrics.ccd_thickness;
        let sweep_radius = fast.size_metrics.sweep_radius;

        // Compute the speed and displacement to determine whether the body moved
        // enough to be considered a fast body that requires CCD.

        // Linear and angular speed
        let lin_speed = body.linear_velocity.length();
        #[cfg(feature = "2d")]
        let ang_speed = body.angular_velocity.abs();
        #[cfg(feature = "3d")]
        let ang_speed = body.angular_velocity.length();

        // Displacement of the center of mass
        let delta_pos_len = body.delta_position.length();

        // The chord length of the rotation. This is a straight-line approximation
        // of the displacement of the farthest point (per unit sweep radius).
        let delta_rot_chord = body.delta_rotation.chord_length();

        // The velocity of the farthest point on the body
        let max_velocity = lin_speed + ang_speed * sweep_radius;

        // The maximum distance the farthest point on the body travels during the timestep
        let max_delta_position = delta_pos_len + delta_rot_chord * sweep_radius;

        // The maximum of the two, which is an upper bound on how far any point on the body travels
        let max_motion = max_delta_position.max(max_velocity * delta_secs);

        // Check if the body moved more than the threshold fraction of its CCD thickness.
        // CCD is only performed for bodies that are actually at risk of tunneling or deep overlap.
        if max_motion <= ccd.threshold * ccd_thickness {
            // Not a fast body, so no CCD is needed.
            return;
        }

        // Records a fast body's time-of-impact fraction and earliest impact on the current thread.
        let record = |fraction, impact| {
            thread_local_results
                .get_or(|| RefCell::new(Vec::new()))
                .borrow_mut()
                .push(CcdResult {
                    entity: fast.entity,
                    fraction,
                    impact,
                });
        };

        let mode1 = ccd.mode;
        let body_com_world = fast.position.0 + (fast.rotation * fast.com.0).real();

        // Determine which trees to sweep against based on the body's filter.
        let trees_to_query: [Option<&ColliderTree>; 4] = [
            ccd.filter
                .contains(CcdFilter::DYNAMIC)
                .then_some(&trees.dynamic_tree),
            ccd.filter
                .contains(CcdFilter::KINEMATIC)
                .then_some(&trees.kinematic_tree),
            ccd.filter
                .contains(CcdFilter::STATIC)
                .then_some(&trees.static_tree),
            ccd.filter
                .contains(CcdFilter::STANDALONE)
                .then_some(&trees.standalone_tree),
        ];

        // The smallest time of impact found across all of the body's colliders.
        // Starts at the full timestep duration.
        let mut min_toi = delta_secs;

        // Details of the earliest impact found across all of the body's colliders.
        let mut best_impact: Option<CcdImpact> = None;

        // Sweep each collider attached to the body.
        for collider_entity in fast.colliders {
            let Ok((collider1, &collider_pos1, &collider_rot1)) = colliders.get(collider_entity)
            else {
                continue;
            };

            let motion1 =
                collider_sweep_motion(collider_pos1.0, collider_rot1, body_com_world, body, inv_dt);

            // Compute the collider's end-of-frame pose to build the swept AABB.
            let collider_rot2 = (body.delta_rotation * Rot::from(collider_rot1)).fast_renormalize();
            let collider_pos2 = body_com_world
                + body.delta_position.real()
                + body.delta_rotation.real() * (collider_pos1.0 - body_com_world);
            let swept_aabb = collider1.swept_aabb(
                collider_pos1.0,
                collider_rot1,
                collider_pos2,
                collider_rot2,
                0.0,
            );
            let query_aabb = obvhs::aabb::Aabb::from(swept_aabb);

            // Sweep the collider against the relevant trees.
            for tree in trees_to_query.into_iter().flatten() {
                tree.aabb_traverse(query_aabb, |proxy_id| {
                    let Some(proxy) = tree.get_proxy(proxy_id) else {
                        return true;
                    };

                    // Skip self-collisions within the same body.
                    if proxy.body == Some(fast.entity) {
                        return true;
                    }

                    // Skip sensors and disabled bodies.
                    if proxy.is_sensor()
                        || proxy.flags.contains(ColliderTreeProxyFlags::BODY_DISABLED)
                    {
                        return true;
                    }

                    let collider_entity2 = proxy.collider;

                    // If the narrow phase already has a touching contact for this pair,
                    // the discrete contact solver is responsible for it.
                    //
                    // Skipping it here is important because clamping the body at the time of impact
                    // would overwrite the already computed correction from the contact solver,
                    // which could lead to the bodies getting frozen. This is not a problem in Box2D
                    // because it runs CCD after updating body poses (but needs to store old poses).
                    if contact_graph
                        .get(collider_entity, collider_entity2)
                        .is_some_and(|(_, pair)| pair.is_touching())
                    {
                        return true;
                    }

                    // Fetch the target collider and its start-of-frame pose.
                    let Ok((collider2, &target_pos, &target_rot)) = colliders.get(collider_entity2)
                    else {
                        return true;
                    };

                    // Reconstruct the target's motion over the timestep.
                    // Bodies with a solver body are awake and moving, everything else is stationary.
                    let (motion2, mode2) = match proxy.body {
                        Some(body2_entity) => match ccd_query.get(body2_entity) {
                            Ok(body2) => {
                                let target_com_world =
                                    body2.position.0 + (body2.rotation * body2.com.0).real();
                                (
                                    collider_sweep_motion(
                                        target_pos.0,
                                        target_rot,
                                        target_com_world,
                                        body2.body,
                                        inv_dt,
                                    ),
                                    body2.ccd.map_or(SweepMode::NonLinear, |ccd| ccd.mode),
                                )
                            }
                            Err(_) => (static_motion(target_pos.0, target_rot), SweepMode::Linear),
                        },
                        None => (static_motion(target_pos.0, target_rot), SweepMode::Linear),
                    };

                    // Use a non-linear sweep unless both bodies opt into linear sweeps.
                    let sweep_mode = if mode1 == SweepMode::Linear && mode2 == SweepMode::Linear {
                        SweepMode::Linear
                    } else {
                        SweepMode::NonLinear
                    };

                    // Compute the time of impact for this pair of colliders. If it's the earliest
                    // so far, record the details needed to clamp the body's normal motion and hand
                    // the contact off to the discrete solver next frame.
                    if let Some(hit) = compute_ccd_toi(
                        sweep_mode, &motion1, collider1, &motion2, collider2, min_toi,
                    ) {
                        min_toi = hit.time_of_impact.f32();
                        best_impact = Some(CcdImpact {
                            collider1: collider_entity,
                            collider2: collider_entity2,
                            // The additional speculative distance only needs to span the gap
                            // the fast contact opens. The body's max travel is a safe upper bound
                            // that the fast-body test already computed. The narrow phase has a more
                            // elaborate velocity test that decides whether to actually keep the contact.
                            distance: max_motion,
                        });
                    }

                    true
                });
            }
        }

        // Record the minimum time of impact as a fraction of the timestep.
        record(min_toi * inv_dt, best_impact);
    });

    // Collect the per-thread results and sort them for determinism.
    let mut results: Vec<CcdResult> = thread_local_results
        .into_iter()
        .flat_map(|cell| cell.into_inner())
        .collect();
    results.sort_unstable_by_key(|result| result.entity);

    // Mark fast bodies and apply any time-of-impact corrections.
    //
    // Also ensure a contact pair exists for any impacts that were found,
    // and request a speculative distance to help ensure the contact is detected
    // by the discrete solver next timestep.
    if !results.is_empty() {
        let mut body_query = bodies.p1();
        for CcdResult {
            entity,
            fraction,
            impact,
        } in results
        {
            if let Ok(mut solver_body) = body_query.get_mut(entity) {
                // Note: This flag is only retained for debug rendering.
                solver_body.flags.insert(SolverBodyFlags::IS_FAST);

                if fraction < 1.0 {
                    // Note: This flag is only retained for debug rendering.
                    solver_body
                        .flags
                        .insert(SolverBodyFlags::HAD_TIME_OF_IMPACT);

                    let t = fraction.min(1.0);

                    // Scale back the body's motion to the time of impact.
                    solver_body.delta_position *= t;
                    #[cfg(feature = "2d")]
                    {
                        solver_body.delta_rotation =
                            Rot2::IDENTITY.nlerp(solver_body.delta_rotation, t);
                    }
                    #[cfg(feature = "3d")]
                    {
                        solver_body.delta_rotation =
                            Quat::IDENTITY.lerp(solver_body.delta_rotation, t);
                    }
                }
            }

            // Request an additional speculative distance for any existing separated contact pair.
            // This helps ensure the contact is detected by the discrete solver next timestep,
            // even if the bodies ended up separated at the TOI impact.
            //
            // This is typically only relevant for dynamic-dynamic and dynamic-kinematic impacts
            // where the final pose of the other shape ended up separated from the TOI impact point.
            // In the `tumbler` example, this manifested as dynamic boxes continuously having a TOI impact
            // against the spinning container, being effectively frozen in place with no proper
            // velocity correction, which also caused significant overlap with other boxes.
            if fraction < 1.0
                && let Some(impact) = impact
                && let Some((_edge, pair)) =
                    contact_graph.get_mut(impact.collider1, impact.collider2)
            {
                pair.speculative_distance = pair.speculative_distance.max(impact.distance);
            }
        }
    }

    diagnostics.swept_ccd += start.elapsed();
}

/// Reconstructs the sweep of a single collider for the timestep from its [`SolverBody`] deltas,
/// as a Parry [`NonlinearRigidMotion`] with per-second velocities.
///
/// `collider_pos`/`collider_rot` are the collider's world pose at the start of the timestep, and
/// `com_world` is its body's center of mass in world space. The body rotates about its center of
/// mass, so this supports colliders offset from the body origin (i.e. child colliders).
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
fn collider_sweep_motion(
    collider_pos: RVector,
    collider_rot: impl Into<Rot>,
    com_world: RVector,
    solver_body: &SolverBody,
    inv_dt: f32,
) -> NonlinearRigidMotion {
    let collider_rot = collider_rot.into();

    let lin_vel = solver_body.delta_position * inv_dt;
    #[cfg(feature = "2d")]
    let ang_vel = solver_body.delta_rotation.as_radians() * inv_dt;
    #[cfg(feature = "3d")]
    let ang_vel = solver_body.delta_rotation.to_scaled_axis() * inv_dt;

    // The body's center of mass expressed in the collider's local frame
    let local_center = collider_rot.inverse() * (com_world - collider_pos).f32();

    NonlinearRigidMotion::new(
        make_pose(collider_pos, collider_rot),
        local_center.real(),
        lin_vel.real(),
        ang_vel.real(),
    )
}

/// A constant (non-moving) [`NonlinearRigidMotion`] at the given pose, for static targets.
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
fn static_motion(pos: RVector, rot: impl Into<Rot>) -> NonlinearRigidMotion {
    NonlinearRigidMotion::constant_position(make_pose(pos, rot))
}

/// Computes the [`ShapeCastHit`] at which `motion1` (sweeping `collider1`) first touches
/// `motion2` (sweeping `collider2`), if any, using the specified `mode`.
///
/// Returns `None` if no impact is found within `min_toi`.
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
fn compute_ccd_toi(
    mode: SweepMode,
    motion1: &NonlinearRigidMotion,
    collider1: &Collider,
    motion2: &NonlinearRigidMotion,
    collider2: &Collider,
    min_toi: f32,
) -> Option<ShapeCastHit> {
    let shape1 = collider1.shape_scaled();
    let shape2 = collider2.shape_scaled();

    let hit = if mode == SweepMode::Linear {
        cast_shapes(
            &motion1.start,
            motion1.linvel,
            shape1.as_ref(),
            &motion2.start,
            motion2.linvel,
            shape2.as_ref(),
            ShapeCastOptions {
                max_time_of_impact: min_toi.real(),
                stop_at_penetration: false,
                ..default()
            },
        )
        .ok()??
    } else {
        cast_shapes_nonlinear(
            motion1,
            shape1.as_ref(),
            motion2,
            shape2.as_ref(),
            0.0,
            min_toi.real(),
            false,
        )
        .ok()??
    };

    (hit.time_of_impact > 0.0 && hit.time_of_impact < min_toi.real()).then_some(hit)
}
