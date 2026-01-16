import pandas as pd
import matplotlib.pyplot as plt

def generate_five_drone_plots(input_csv):
    # 1. Wczytanie danych
    df = pd.read_csv(input_csv)
    
    # Czyszczenie nazw kolumn (usunięcie spacji przed/po nazwie)
    df.columns = df.columns.str.strip()
    
    # Oś X - przyjmujemy pierwszą kolumnę (indeks/czas)
    x = df.iloc[:, 0]
    
    # Ustawienie wspólnego stylu
    # plt.style.use('seaborn-v0_8-grid') # lub 'ggplot'

    # --- Wykres 1: Total Drones, Active Drones, Collisions ---
    plt.figure(figsize=(10, 5))
    plt.plot(x, df['Total Drones'], label='Total Drones', linewidth=2)
    plt.plot(x, df['Active Drones'], label='Active Drones', linestyle='--')
    plt.plot(x, df['Collisions'], label='Collisions', color='red')
    plt.title('Flota i Kolizje')
    plt.xlabel('Tick')
    plt.ylabel('Liczba')
    plt.legend()
    plt.savefig('1_drony_i_kolizje.png')
    plt.close()

    # --- Wykres 2: Completed Deliveries ---
    plt.figure(figsize=(10, 5))
    plt.plot(x, df['Completed Deliveries'], color='green', label='Zrealizowane dostawy')
    plt.title('Ukończone dostawy')
    plt.xlabel('Tick')
    plt.ylabel('Suma dostaw')
    plt.legend()
    plt.savefig('2_dostawy.png')
    plt.close()

    # --- Wykres 3: Avg Delivery Time [minutes] ---
    plt.figure(figsize=(10, 5))
    plt.plot(x, df['Avg Delivery Time [minutes]'], color='orange', label='Średni czas dostawy')
    plt.title('Czas dostawy (min)')
    plt.xlabel('Tick')
    plt.ylabel('Minuty')
    plt.legend()
    plt.savefig('3_czas_dostawy.png')
    plt.close()

    # --- Wykres 4: Deliveries Per Minute ---
    plt.figure(figsize=(10, 5))
    plt.plot(x, df['Deliveries Per Minute'], color='purple', label='Wydajność (dostawy/min)')
    plt.title('Dostawy na minutę')
    plt.xlabel('Tick')
    plt.ylabel('Dostawy/Min')
    plt.legend()
    plt.savefig('4_wydajnosc.png')
    plt.close()

    # --- Wykres 5: Time Per Meter [seconds] ---
    plt.figure(figsize=(10, 5))
    plt.plot(x, df['Time Per Meter [seconds]'], color='brown', label='Czas na metr')
    plt.title('Czas przelotu jednego metra (sekundy)')
    plt.xlabel('Tick')
    plt.ylabel('Sekundy/Metr')
    plt.legend()
    plt.savefig('5_czas_na_metr.png')
    plt.close()

    print("Wygenerowano 5 wykresów.")

# Przykład użycia:
generate_five_drone_plots('run_2026_01_15__23_47_46/model_data.csv')