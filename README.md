# LDACS-Abstract-TDMA-MAC

## Overview
LDACS-Abstract-TDMA-MAC is part of the LDACS-Greedy-K-Hop-Simulator project. It implements an abstract version of the MCSOTDMA MAC protocol designed for LDACS A2A communications. This component models the logic for managing a Shared (SH) channel and multiple Point-to-Point (PP) channels, utilizing two receiving antennas and one transmitting antenna. The simulation simplifies the link establishment process by assuming a central entity performs optimal scheduling for both SH and PP channels. For more information on the MCSOTDMA specification, visit [MCSOTDMA Specification](https://github.com/ComNetsHH/ldacs_mcsotdma_specification).

## Installation
Clone the repository and follow the setup instructions:
```bash
git clone https://github.com/ComNetsHH/LDACS-Abstract-TDMA-MAC.git
cd LDACS-Abstract-TDMA-MAC/simulation
make build-release
```