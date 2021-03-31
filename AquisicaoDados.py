import csv
import serial  # importa a biblioteca pyserial
import datetime
import sys
import time

# insira toda a inicialização aqui
start = "true"
# abre a  porta disponível

try:
    ser = serial.Serial("COM4", timeout=10)
    time.sleep(3)
    ser.write ("OK".encode('ascii'))
    start = ser.readline().decode(encoding='utf-8')
    print(start)
    if start == "OK":
        time.sleep(2.5)
        start = 'true'
    else:
        start = 'false'
        i = 0
    try:
        with open( 'dados_balança.csv', 'w', newline='' ) as csv_file:
            csv_writer = csv.DictWriter( csv_file, fieldnames=["ID", "DATA", "EIXO X", "EIXO Y", "EIXO Z", \
                                                           "TENSÃO", "CORRENTE", "TEMPERATURA", "GIRO X", "GIRO Y",
                                                           "GIRO Z"], delimiter=';')
            csv_writer.writeheader()
    except:
        print("ERRO AO CRIAR O ARQUIVO")
    while start:
        while ser.inWaiting() > 0:
            try:
                caracLido = ser.readline()
                frase = caracLido.decode(encoding='utf-8').split(";")
                #print(frase)
                EixoX = (frase[1]).strip(' ')
                EixoXfloat = (frase[1]).replace('.',',')
                EixoY = (frase[3]).strip(' ')
                EixoYfloat = (frase[3]).replace('.', ',')
                EixoZ = (frase[5]).strip(' ')
                EixoZfloat = (frase[5]).replace('.', ',')
                tensao = (frase[7]).strip(' ')
                tensaofloat = (frase[7]).replace('.', ',')
                corrente = (frase[9]).strip(' ')
                correntefloat = (frase[9]).replace('.', ',')
                temperatura = (frase[11]).strip(' ')
                temperaturafloat = (frase[11]).replace('.', ',')
                GiroX = (frase[13]).strip(' ')
                giroXfloat = (frase[13]).replace('.', ',')
                GiroY = (frase[15]).strip(' ')
                giroYfloat = (frase[15]).replace('.', ',')
                GiroZ = (frase[17]).strip(' ')
                giroZfloat = (frase[17]).replace('.', ',')
                print(' Eixo X:',EixoXfloat," Eixo Y:",EixoYfloat," Eixo Z:",EixoZfloat,
                      " Tensão:",tensaofloat," Corrente:",correntefloat," Tempetarura:",
                temperaturafloat," Giro X:",giroXfloat," Giro Y:",
                giroYfloat," Giro Z:",giroZfloat);
                i = 1 + i
                print('Quantidade de Leituras: ',i)
                x = datetime.datetime.now()
                ano = x.strftime("%Y")
                mes = x.strftime("%m")
                dia =x.strftime("%d")
                hora = x.strftime("%H")
                minuto = x.strftime("%M")
                sec = x.strftime("%S")
                DATA= ((dia+'/'+mes+"/"+ano+"  "+hora+":"+minuto+":"+sec))
                print(DATA)

                try:
                    with open( 'dados_balança.csv', 'a', newline='' ) as csv_file:
                        csv_writer = csv.DictWriter( csv_file, fieldnames=["ID", "DATA", "EIXO X", "EIXO Y", "EIXO Z", \
                                                                           "TENSÃO", "CORRENTE", "TEMPERATURA",
                                                                           "GIRO X", "GIRO Y",
                                                                           "GIRO Z"], delimiter=';' )

                        csv_writer.writerow( {'ID':i, 'DATA': DATA,'EIXO X':EixoXfloat,"EIXO Y":EixoYfloat,
                                          'EIXO Z':EixoZfloat,"TENSÃO":tensaofloat,'CORRENTE':correntefloat,
                                          'TEMPERATURA':temperaturafloat,'GIRO X':giroXfloat,'GIRO Y': giroYfloat,
                                          'GIRO Z':giroZfloat})
                        csv_file.close()
                except:
                    print("ERRO AO SALVAR OS ARQUIVOS")
            except:
                print("Erro na leitura ou conversao de valores")
            # Cria a interface TK


except serial.SerialException as e:
    sys.stderr.write("Impossivel abrir porta  %r: %s\n" % (port, e))
    ser.write("NOK")
    start = false
    ser.close()
    sys.exit(1)
